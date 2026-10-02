"""Input failures must reach stderr and the process exit status."""
from pathlib import Path
from queue import Queue
from types import SimpleNamespace
import logging
import subprocess
import sys

import pytest
import yaml

from src.core.thread_control import ThreadControl
from src.core.gnss.rtklib.rinex import rnx_decode
from src.stream.base import StreamerBase
from src.stream.imu_sensor import ImuSensor
from src.stream.formators import ImuFormator, EuRoCImuFormator, PosSolFormator
from src.stream.awesome_sensor import AwesomeImuFormator, AwesomeGnssFormator
from src.core.gnss.rtklib.rtkcmn import rSIG, uGNSS
from src.utility.rinex_improve import improve_rinex
from src.utility.gnutlib import parse_obs_header, iter_obs_epochs

ROOT = Path(__file__).resolve().parents[1]


def rinex_header(version, kind="O", end=True):
    text = f"{version:9.2f}           {kind:<40}RINEX VERSION / TYPE\n"
    if end:
        text += " " * 60 + "END OF HEADER\n"
    return text


@pytest.mark.parametrize("version", [2.11, 3.01, 4.00])
@pytest.mark.parametrize("kind", ["O", "N"])
def test_decoder_rejects_unsupported_version_with_filename(tmp_path, version, kind):
    path = tmp_path / "unsupported.rnx"
    path.write_text(rinex_header(version, kind))
    decoder = rnx_decode(SimpleNamespace(sig_tbl={}, skip_sig_tbl={}))
    with pytest.raises(ValueError) as exc:
        if kind == "O":
            decoder.decode_obsfile(SimpleNamespace(), str(path), None)
        else:
            decoder.decode_nav(str(path), SimpleNamespace())
    assert str(path) in str(exc.value)
    assert f"{version:.2f}" in str(exc.value)
    if decoder.fobs is not None:
        assert decoder.fobs.closed


@pytest.mark.parametrize("text", ["", "not rinex\n", rinex_header(3.04, end=False)])
def test_decoder_rejects_broken_header(tmp_path, text):
    path = tmp_path / "broken.obs"
    path.write_text(text)
    decoder = rnx_decode(SimpleNamespace(sig_tbl={}, skip_sig_tbl={}))
    with pytest.raises(ValueError, match="broken.obs"):
        decoder.decode_obsfile(SimpleNamespace(), str(path), None)


def test_bad_navigation_number_is_not_zero():
    decoder = rnx_decode(SimpleNamespace(sig_tbl={}, skip_sig_tbl={}))
    with pytest.raises(ValueError):
        decoder.flt("not-a-number")
    assert decoder.flt(" " * 19) == 0.0
    assert decoder.flt("1.25D+02") == 125.0


def navigation_file(tmp_path, month=8, blank=False):
    path = tmp_path / "sample.nav"
    clock = f"G01 2025 {month:02d} 28 00 00 00" + f"{0.0:19.12E}" * 3 + "\n"
    continuation = ("\n" if blank else "    " + f"{1.0:19.12E}" * 4 + "\n")
    path.write_text(rinex_header(3.04, "N") + clock + continuation * 7)
    return path


def test_blank_mandatory_navigation_fields_are_not_zero(tmp_path):
    path = navigation_file(tmp_path, blank=True)
    decoder = rnx_decode(SimpleNamespace(sig_tbl={}, skip_sig_tbl={}))
    with pytest.raises(ValueError, match="sample.nav"):
        decoder.decode_nav(str(path), SimpleNamespace())


def test_invalid_navigation_calendar_is_rejected(tmp_path):
    path = navigation_file(tmp_path, month=13)
    decoder = rnx_decode(SimpleNamespace(sig_tbl={}, skip_sig_tbl={}))
    with pytest.raises(ValueError, match="sample.nav"):
        decoder.decode_nav(str(path), SimpleNamespace())


def test_navigation_optional_spare_fields_can_remain_blank(tmp_path):
    path = navigation_file(tmp_path)
    lines = path.read_text().splitlines(keepends=True)
    lines[2] = lines[2][:61] + " " * 19 + "\n"  # optional clock drift-rate
    lines[-1] = lines[-1][:23] + "\n"  # optional fit/spare fields
    path.write_text("".join(lines))
    decoder = rnx_decode(SimpleNamespace(sig_tbl={}, skip_sig_tbl={}))
    nav = SimpleNamespace()
    decoder.decode_nav(str(path), nav)
    assert len(nav.eph) == 1
    assert nav.eph[0].f2 == 0.0


@pytest.mark.parametrize("parser,row", [
    (ImuFormator, "2381,371761,broken,0,0,0,0,9.8"),
    (EuRoCImuFormator, "1756365319504000000,0,0,nan,0,0,9.8"),
    (EuRoCImuFormator, "1756365319504000000,0"),
    (PosSolFormator, "2025/08/28 07:15:38.0 1 2 bad 1 9 0.1 0.1 0.1"),
    (AwesomeImuFormator, "371761 0 bad 0 0 0 0"),
    (AwesomeGnssFormator, "371761 30 114 nan 0.1 0.1 0.1"),
])
def test_malformed_data_row_raises(parser, row):
    with pytest.raises(ValueError):
        parser().decode(row)


def test_imu_thread_reports_filename_line_and_stops(tmp_path, caplog):
    path = tmp_path / "bad.csv"
    path.write_text("# comment\n1756365319504000000,bad,0,0,0,0,9.8\n")
    control = ThreadControl()
    sensor = ImuSensor(str(path), Queue(), control, imu_format="euroc")
    with caplog.at_level(logging.ERROR):
        sensor.start()
        sensor.join(timeout=2)
    assert not sensor.is_alive()
    assert not control.is_running()
    with pytest.raises(RuntimeError) as exc:
        control.raise_if_failed()
    assert str(path) in str(exc.value)
    assert "2" in str(exc.value)
    assert str(path) in caplog.text


def test_empty_external_file_is_reported(tmp_path, caplog):
    path = tmp_path / "empty.pos"
    path.write_text("% header\n\n")
    control = ThreadControl()
    sensor = StreamerBase(str(path), PosSolFormator(), Queue(), control, "gnss")
    with caplog.at_level(logging.ERROR):
        sensor.run()
    with pytest.raises(RuntimeError, match="empty.pos"):
        control.raise_if_failed()
    assert "empty.pos" in caplog.text


def run_cli(tmp_path, config):
    path = tmp_path / "config.yaml"
    path.write_text(yaml.safe_dump(config), encoding="utf-8")
    return subprocess.run([sys.executable, "src/main.py", str(path)],
                          cwd=ROOT, text=True, capture_output=True, timeout=15)


def test_cli_rejects_old_rinex_before_opening_output(tmp_path):
    obs = tmp_path / "rover.obs"
    nav = tmp_path / "nav.rnx"
    obs.write_text(rinex_header(3.01))
    nav.write_text(rinex_header(3.04, "N"))
    output = tmp_path / "RTK.pos"
    output.write_text("previous result")
    result = run_cli(tmp_path, {
        "gnss": {"gnss_source": "internal", "positioning_mode": "spp",
                 "rover_path": str(obs), "eph_path": str(nav)},
        "ins": {"enabled": "off"},
        "output": {"output_dir": str(tmp_path)},
    })
    assert result.returncode != 0
    assert str(obs) in result.stderr
    assert "3.01" in result.stderr
    assert output.read_text() == "previous result"


def test_cli_propagates_imu_thread_failure(tmp_path):
    imu = tmp_path / "bad.csv"
    pos = tmp_path / "gnss.pos"
    imu.write_text("# header\n1756365319504000000,bad,0,0,0,0,9.8\n")
    pos.write_text("2025/08/28 07:15:38.0 -2267041 5006693 3225621 1 9 0.1 0.1 0.1\n")
    result = run_cli(tmp_path, {
        "gnss": {"gnss_source": "external", "external_sol_path": str(pos)},
        "ins": {"enabled": "lc", "data_rate": 100, "imu_format": "euroc",
                "imu_data_path": str(imu)},
        "output": {"output_dir": str(tmp_path / "output")},
    })
    assert result.returncode != 0
    assert str(imu) in result.stderr
    assert "line 2" in result.stderr
    assert "运行时长" not in result.stdout


def test_cli_writer_failure_is_nonzero_and_does_not_hang(tmp_path):
    imu = tmp_path / "imu.csv"
    pos = tmp_path / "gnss.pos"
    output = tmp_path / "not-a-directory"
    output.write_text("keep this file")
    imu.write_text("1756365338000000000,0,0,0,0,0,9.8\n" * 500)
    pos.write_text("2025/08/28 07:15:38.0 -2267041 5006693 3225621 1 9 0.1 0.1 0.1\n")
    result = run_cli(tmp_path, {
        "gnss": {"gnss_source": "external", "external_sol_path": str(pos)},
        "ins": {"enabled": "lc", "data_rate": 100, "imu_format": "euroc",
                "imu_data_path": str(imu)},
        "output": {"output_dir": str(output)},
    })
    assert result.returncode != 0
    assert str(output) in result.stderr.replace("\\\\", "\\")
    assert output.read_text() == "keep this file"


def test_cli_rejects_empty_imu_before_starting_threads(tmp_path):
    imu = tmp_path / "empty.csv"
    pos = tmp_path / "gnss.pos"
    imu.write_text("")
    pos.write_text("% header\n")
    result = run_cli(tmp_path, {
        "gnss": {"gnss_source": "external", "external_sol_path": str(pos)},
        "ins": {"enabled": "lc", "data_rate": 100, "imu_format": "euroc",
                "imu_data_path": str(imu)},
        "output": {"output_dir": str(tmp_path / "output")},
    })
    assert result.returncode != 0
    assert str(imu) in result.stderr
    assert not (tmp_path / "output").exists()


def test_cli_checks_imu_tail_past_last_gnss_epoch(tmp_path):
    imu = tmp_path / "tail.csv"
    pos = tmp_path / "gnss.pos"
    imu.write_text("".join(
        f"{1756365320000000000 + i * 10000000},0,0,0,0,0,9.8\n"
        for i in range(10000)) + "1756365420000000000,bad,0,0,0,0,9.8\n")
    pos.write_text("2025/08/28 07:15:38.0 -2267041 5006693 3225621 1 9 0.1 0.1 0.1\n")
    result = run_cli(tmp_path, {
        "gnss": {"gnss_source": "external", "external_sol_path": str(pos)},
        "ins": {"enabled": "lc", "data_rate": 100, "imu_format": "euroc",
                "imu_data_path": str(imu)},
        "output": {"output_dir": str(tmp_path / "output")},
    })
    assert result.returncode != 0
    assert str(imu) in result.stderr
    assert "10001" in result.stderr
    assert not (tmp_path / "output").exists()


def test_cli_valid_external_input_still_completes(tmp_path):
    imu = tmp_path / "imu.csv"
    pos = tmp_path / "gnss.pos"
    imu.write_text("".join(
        f"{1756365320000000000 + i * 10000000},0,0,0,0,0,9.8\n"
        for i in range(200)))
    pos.write_text("2025/08/28 07:15:38.0 -2267041 5006693 3225621 1 9 0.1 0.1 0.1\n")
    result = run_cli(tmp_path, {
        "gnss": {"gnss_source": "external", "external_sol_path": str(pos)},
        "ins": {"enabled": "lc", "data_rate": 100, "imu_format": "euroc",
                "imu_data_path": str(imu)},
        "output": {"output_dir": str(tmp_path / "output")},
    })
    assert result.returncode == 0, result.stderr
    assert "ERROR" not in result.stderr
    assert (tmp_path / "output/aligned.csv").is_file()


def observation_file(tmp_path, corrupt=False):
    path = tmp_path / "sample.obs"
    codes = "G    4 C1C L1C D1C S1C".ljust(60) + "SYS / # / OBS TYPES\n"
    header = rinex_header(3.04, end=False) + codes + " " * 60 + "END OF HEADER\n"
    number = "bad-value" if corrupt else "20479285.686"
    row = "G01 " + number.rjust(13) + "   " + " " * 16 + f"{-2787.94:13.3f}   {33.0:13.3f}   \n"
    path.write_text(header + "> 2025  8 28  7 15 38.0000000  0  1\n" + row)
    return path


def test_valid_rinex_keeps_pseudorange_and_legal_missing_phase(tmp_path):
    path = observation_file(tmp_path)
    decoder = rnx_decode(SimpleNamespace(sig_tbl={"1C": rSIG.L1C},
                                         skip_sig_tbl={uGNSS.GPS: []}),
                         raw_band_priority={"G": [1]})
    decoder.decode_obsfile(SimpleNamespace(gnss_t=[uGNSS.GPS]), str(path), None)
    assert len(decoder.obslist) == 1
    assert decoder.obslist[0].P[0, 0] == pytest.approx(20479285.686)
    assert decoder.obslist[0].L[0, 0] == 0.0
    assert decoder.fobs.closed


def test_invalid_observation_calendar_is_rejected(tmp_path):
    path = observation_file(tmp_path)
    path.write_text(path.read_text().replace("> 2025  8 28", "> 2025 13 28"))
    decoder = rnx_decode(SimpleNamespace(sig_tbl={"1C": rSIG.L1C},
                                         skip_sig_tbl={uGNSS.GPS: []}),
                         raw_band_priority={"G": [1]})
    with pytest.raises(ValueError, match="sample.obs"):
        decoder.decode_obsfile(SimpleNamespace(gnss_t=[uGNSS.GPS]), str(path), None)


def test_preprocessing_reports_bad_observation_original_path_and_line(tmp_path):
    path = observation_file(tmp_path, corrupt=True)
    with pytest.raises(ValueError) as exc:
        improve_rinex(str(path), str(tmp_path / "prepared.obs"),
                      band_plan={"G": [1]}, gnss_t=["GPS"])
    assert str(path) in str(exc.value)
    assert "line 5" in str(exc.value)


def test_preprocessing_rejects_truncated_epoch(tmp_path):
    path = observation_file(tmp_path)
    path.write_text(path.read_text().rsplit("G01 ", 1)[0])
    header = parse_obs_header(str(path))
    with pytest.raises(ValueError, match="truncated"):
        list(iter_obs_epochs(str(path), header))


def test_phase_shift_header_does_not_parse_label_as_numeric(tmp_path):
    path = observation_file(tmp_path)
    shift = "G L1C -0.25000".ljust(60) + "SYS / PHASE SHIFT\n"
    text = path.read_text().replace(" " * 60 + "END OF HEADER\n",
                                    shift + " " * 60 + "END OF HEADER\n")
    path.write_text(text)
    header = parse_obs_header(str(path))
    assert header.phase_shifts["G"]["L1C"] == -0.25


def test_cancellation_releases_full_producer_queue(tmp_path):
    path = tmp_path / "imu.csv"
    path.write_text("1756365319504000000,0,0,0,0,0,9.8\n" * 5)
    queue = Queue(maxsize=1)
    control = ThreadControl()
    sensor = ImuSensor(str(path), queue, control, imu_format="euroc")
    sensor.start()
    queue.get(timeout=2)
    control.fail("consumer", ValueError("cannot write output"))
    sensor.join(timeout=2)
    assert not sensor.is_alive()
    with pytest.raises(RuntimeError, match="cannot write output"):
        control.raise_if_failed()


def test_missing_file_reports_its_path(tmp_path, caplog):
    path = tmp_path / "missing.pos"
    control = ThreadControl()
    sensor = StreamerBase(str(path), PosSolFormator(), Queue(), control, "gnss")
    with caplog.at_level(logging.ERROR):
        sensor.run()
    with pytest.raises(RuntimeError, match="missing.pos"):
        control.raise_if_failed()
    assert str(path) in caplog.text


@pytest.mark.parametrize("parser, comment", [
    (ImuFormator, "# header"), (EuRoCImuFormator, "# header"),
    (PosSolFormator, "% header"), (AwesomeImuFormator, "# header"),
    (AwesomeGnssFormator, "# header"),
])
def test_comments_and_blank_lines_remain_legal(parser, comment):
    assert parser().decode(comment) is None
    assert parser().decode("  \n") is None
