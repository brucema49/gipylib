"""
module for RINEX 3.0x processing

Copyright (c) 2021 Rui Hirokawa (from CSSRLIB)
Copyright (c) 2022 Tim Everett
"""

import numpy as np
from copy import deepcopy
from .rtkcmn import uGNSS, rSIG, Eph, Geph, prn2sat, gpst2time, time2gpst, Obs, \
                    epoch2time, timediff, timeadd, utc2gpst
from . import rtkcmn as gn
from .ephemeris import satposs
from src.stream.gnss_band_mapping import (
    DEFAULT_RAW_BAND_PRIORITY,
    raw_band_priority_to_slot_mapping,
)

# RINEX 观测量按"类型分组"排列时 (如 BASE: C,C,L,L,S,S), 位置分块启发式失效。
# 此处以信号频带号为主键映射到频点槽位, 与 freq_ix 配置语义一致:
#   GPS/GLO: L1->0, L2->1        GAL: E1(band1)->0, E5b(band7)->1
#   BDS: B1I/B1C(band1)->0, B3I(band6)->1
# Kept as a compatibility view for callers that imported the old constant.
# ``rnx_decode`` deliberately does not read this global: each decoder owns its
# raw-band mapping and can therefore represent a different stream layout.
BAND_SLOT = {
    uGNSS.GPS: {1: 0, 2: 1},
    uGNSS.GLO: {1: 0, 2: 1},
    uGNSS.GAL: {1: 0, 7: 1},
    uGNSS.BDS: {1: 0, 6: 1},
    uGNSS.QZS: {1: 0, 2: 1},
}


class rnx_decode:
    """ class for RINEX decoder """
    MAXSAT = uGNSS.GPSMAX+uGNSS.GLOMAX+uGNSS.GALMAX+uGNSS.BDSMAX+uGNSS.QZSMAX

    def __init__(self, cfg, raw_band_priority=None):
        self.ver = -1.0
        self.fobs = None
        self.gnss_tbl = {'G': uGNSS.GPS, 'E': uGNSS.GAL, 'R': uGNSS.GLO, 'J': uGNSS.QZS, 'C': uGNSS.BDS}
        if raw_band_priority is None:
            raw_band_priority = DEFAULT_RAW_BAND_PRIORITY
        # The mapping is immutable from the decoder's perspective and is
        # derived per instance, never shared through BAND_SLOT.
        self.raw_band_to_slot = raw_band_priority_to_slot_mapping(
            raw_band_priority
        )
        self.slot_frequency_hz = self._slot_frequency_hz(cfg)
        self.sig_tbl = cfg.sig_tbl
        self.skip_sig_tbl = cfg.skip_sig_tbl
        self.nf = 4
        self.sigid = np.ones((uGNSS.GNSSMAX, rSIG.SIGMAX*3), dtype=int) * rSIG.NONE
        self.sigband = np.zeros((uGNSS.GNSSMAX, rSIG.SIGMAX*3), dtype=int)
        self.typeid = np.ones((uGNSS.GNSSMAX, rSIG.SIGMAX*3), dtype=int) * rSIG.NONE
        self.nsig = np.zeros((uGNSS.GNSSMAX), dtype=int)
        self.nband = np.zeros((uGNSS.GNSSMAX), dtype=int)
        self.pos = np.array([0, 0, 0])

    def _slot_frequency_hz(self, cfg):
        """Expose configured solver Hz for this decoder's ordered slots.

        Raw-band selection and physical frequencies are separate namespaces:
        this diagnostic view pairs slot order with the existing solver
        ``freq_ix0``/``freq_ix1`` metadata without deriving either from the
        other.  Minimal decoder configs used by reader-only callers simply
        produce an empty view.
        """
        freq = getattr(cfg, "freq", None)
        if freq is None:
            return {}
        index_tables = [
            getattr(cfg, "freq_ix0", {}),
            getattr(cfg, "freq_ix1", {}),
        ]
        result = {}
        for system, bands in self.raw_band_to_slot.items():
            if system not in self.gnss_tbl:
                continue
            enum = self.gnss_tbl[system]
            values = []
            for table in index_tables[:len(bands)]:
                if enum not in table:
                    break
                index = table[enum]
                if index < 0 or index >= len(freq):
                    break
                values.append(float(freq[index]))
            if values:
                result[system] = values
        return result

    def flt(self, u, c=-1):
        if c >= 0:
            u = u[19*c+4:19*(c+1)+4]
        try:
            return float(u.replace("D", "E"))
        except:
            return 0
        
    
    def adjday(self, t, t0):
        """" adjust time considering week handover  """
        tt = timediff(t, t0)
        if tt < -43200.0:
            return timeadd(t, 86400.0)
        if tt > 43200.0:
            return timeadd(t,-86400.0)
        return t

    def decode_nav(self, navfile, nav):
        """decode RINEX Navigation message from file """
        nav.eph = []
        nav.geph = []
        with open(navfile, 'rt') as fnav:
            for line in fnav:
                if line[60:73] == 'END OF HEADER':
                    break
                elif line[60:80] == 'RINEX VERSION / TYPE':
                    self.ver = float(line[4:10])
                    if self.ver < 3.02:
                        return -1
                elif line[60:76] == 'IONOSPHERIC CORR':
                    if line[0:4] == 'GPSA' or line[0:4] == 'QZSA':
                        for k in range(4):
                            nav.ion[0, k] = self.flt(line[5+k*12:5+(k+1)*12])
                    if line[0:4] == 'GPSB' or line[0:4] == 'QZSB':
                        for k in range(4):
                            nav.ion[1, k] = self.flt(line[5+k*12:5+(k+1)*12])

            for line in fnav:
                if line[0] not in self.gnss_tbl:
                    continue
                sys = self.gnss_tbl[line[0]]
                prn = int(line[1:3])
                if sys == uGNSS.QZS:
                    prn += 192
                sat = prn2sat(sys, prn)
                year = int(line[4:8])
                month = int(line[9:11])
                day = int(line[12:14])
                hour = int(line[15:17])
                minute = int(line[18:20])
                sec = int(line[21:23])
                toc = epoch2time([year, month, day, hour, minute, sec])
                if sys != uGNSS.GLO:
                    eph = Eph(sat)
                    eph.toc = toc
                    eph.f0 = self.flt(line, 1)
                    eph.f1 = self.flt(line, 2)
                    eph.f2 = self.flt(line, 3)
    
                    line = fnav.readline() #3:6
                    eph.iode = int(self.flt(line, 0)) 
                    eph.crs = self.flt(line, 1)
                    eph.deln = self.flt(line, 2)
                    eph.M0 = self.flt(line, 3)
    
                    line = fnav.readline() #7:10
                    eph.cuc = self.flt(line, 0)
                    eph.e = self.flt(line, 1)
                    eph.cus = self.flt(line, 2)
                    sqrtA = self.flt(line, 3)
                    eph.A = sqrtA**2
    
                    line = fnav.readline() #11:14
                    eph.toes = int(self.flt(line, 0))
                    eph.cic = self.flt(line, 1)
                    eph.OMG0 = self.flt(line, 2)
                    eph.cis = self.flt(line, 3)
    
                    line = fnav.readline() #15:18
                    eph.i0 = self.flt(line, 0)
                    eph.crc = self.flt(line, 1)
                    eph.omg = self.flt(line, 2)
                    eph.OMGd = self.flt(line, 3)
    
                    line = fnav.readline() #19:22
                    eph.idot = self.flt(line, 0)
                    eph.code = int(self.flt(line, 1))  # source for GAL NAV type
                    eph.week = int(self.flt(line, 2))
    
                    line = fnav.readline() #23:26
                    eph.sva = self.flt(line, 0)
                    eph.svh = int(self.flt(line, 1))
                    tgd = np.zeros(2)
                    tgd[0] = float(self.flt(line, 2))
                    if sys == uGNSS.GAL:
                        tgd[1] = float(self.flt(line, 3))
                    else:
                        eph.iodc = int(self.flt(line, 3))
                    eph.tgd = tgd
    
                    line = fnav.readline() #27:30
                    tot = int(self.flt(line, 0))
                    if len(line) >= 42:
                        eph.fit = int(self.flt(line, 1))
    
                    # BDS ephemeris uses BDT week (epoch 2006-01-01 = GPS week 1356)
                    # Convert to GPS week for correct toe/tot computation
                    # BDT = GPST - 14s: RINEX BDS 时间标签为 BDT, 需加 14s 转 GPST
                    # (RTKLIB C rinex.c:1252-1258 bdt2gpst, 缺失会导致 tk 偏 14s,
                    #  MEO 卫星位置沿迹误差 ~50km, SPP 发散而 RTK 双差不受影响)
                    if sys == uGNSS.BDS:
                        eph.week += 1356
                        eph.toc = timeadd(eph.toc, 14.0)
                        eph.toe = timeadd(gpst2time(eph.week, eph.toes), 14.0)
                        eph.tot = timeadd(gpst2time(eph.week, tot), 14.0)
                    else:
                        eph.toe = gpst2time(eph.week, eph.toes)
                        eph.tot = gpst2time(eph.week, tot)
                    nav.eph.append(eph)
                else:  # GLONASS
                    if prn > uGNSS.GLOMAX:
                        print('Reject nav entry: %s' % line[:3])
                        break
                    geph = Geph(sat)
                    # Toc rounded by 15 min in utc 
                    week, tow = time2gpst(toc)
                    toc = gpst2time(week,np.floor((tow + 450.0) / 900.0) * 900)
                    dow = int(np.floor(tow  / 86400.0))
                    # time of frame in UTC 
                    tod = self.flt(line, 2) % 86400
                    tof = gpst2time(week ,tod + dow * 86400.0)
                    tof = self.adjday(tof, toc)
                    geph.toe = utc2gpst(toc)
                    geph.tof = utc2gpst(tof)
                    # IODE = Tb (7bit), Tb =index of UTC+3H within current day
                    geph.iode = int(((tow + 10800.0) % 86400) / 900.0 + 0.5)
                    geph.taun = -self.flt(line, 1)
                    geph.gamn = self.flt(line, 2)
                    
                    line = fnav.readline() #3:6
                    pos =np.zeros(3)
                    vel = np.zeros(3)
                    acc = np.zeros(3)
                    pos[0] = self.flt(line, 0)
                    vel[0] = self.flt(line, 1)
                    acc[0] = self.flt(line, 2)
                    geph.svh = self.flt(line, 3)
                    
                    line = fnav.readline() #7:10
                    pos[1] = self.flt(line, 0)
                    vel[1] = self.flt(line, 1)
                    acc[1] = self.flt(line, 2)
                    geph.frq = self.flt(line, 3)
                    nav.glofrq[sat - uGNSS.GPSMAX - 1] = int(geph.frq)

                    line = fnav.readline() #11:14
                    pos[2] = self.flt(line, 0)
                    vel[2] = self.flt(line, 1)
                    acc[2] = self.flt(line, 2)                                      
                    geph.age = self.flt(line, 2)
                    
                    geph.pos = pos * 1000
                    geph.vel = vel * 1000
                    geph.acc = acc * 1000
                    
                    nav.geph.append(geph)
    
        #nav.eph.sort(key=lambda x: (x.sat, x.toe.time))
        nav.eph.sort(key=lambda x: x.toe.time)
        nav.geph.sort(key=lambda x: x.toe.time)
        return nav

    def decode_obsh(self, obsfile):
        self.fobs = open(obsfile, 'rt')
        for line in self.fobs:
            if line[60:73] == 'END OF HEADER':
                break
            if line[60:80] == 'RINEX VERSION / TYPE':
                self.ver = float(line[4:10])
                if self.ver < 3.02:
                    return -1
            elif line[60:79] == 'APPROX POSITION XYZ':
                self.pos = np.array([float(line[0:14]),
                                     float(line[14:28]),
                                     float(line[28:42])])
            elif line[60:79] == 'SYS / # / OBS TYPES':
                if line[0] in self.gnss_tbl:
                    sys = self.gnss_tbl[line[0]]
                else:
                    continue
                self.nsig[sys] = int(line[3:6])
                s = line[7:7+4*13]
                if self.nsig[sys] >= 14:
                    line2 = self.fobs.readline()
                    s += line2[7:7+4*13]

                for k in range(self.nsig[sys]):
                    sig = s[4*k:3+4*k]
                    if sig[1:3] not in self.sig_tbl:
                        continue
                    if self.sig_tbl[sig[1:3]] in self.skip_sig_tbl[sys]:
                        continue
                    if sig[0] == 'C':
                        self.typeid[sys][k] = 0
                    elif sig[0] == 'L':
                        self.typeid[sys][k] = 1
                    elif sig[0] == 'S':
                        self.typeid[sys][k] = 2
                    elif sig[0] == 'D':
                        self.typeid[sys][k] = 3
                    else:
                        continue
                    self.sigid[sys][k] = self.sig_tbl[sig[1:3]]
                    # 记录每列的 RINEX 频带号 (sig 第二字符), 供列->频点槽位映射
                    self.sigband[sys][k] = int(sig[1]) if sig[1].isdigit() else -1
                self.nband[sys] = len(np.where(self.typeid[sys]==1)[0])
        return 0

    def decode_obs(self, nav, maxepoch):
        """decode RINEX Observation message from file """

        self.obslist = []
        nepoch = 0
        for line in self.fobs:
            if line == '':
                break
            if line[0] != '>':
                continue
            obs = Obs()
            nsat = int(line[32:35])
            year = int(line[2:6])
            month = int(line[7:9])
            day = int(line[10:12])
            hour = int(line[13:15])
            minute = int(line[16:18])
            sec = float(line[19:29])
            obs.t = epoch2time([year, month, day, hour, minute, sec])
            obs.P = np.zeros((nsat, gn.MAX_NFREQ))
            obs.L = np.zeros((nsat, gn.MAX_NFREQ))
            obs.D = np.zeros((nsat, gn.MAX_NFREQ))
            obs.S = np.zeros((nsat, gn.MAX_NFREQ))
            obs.lli = np.zeros((nsat, gn.MAX_NFREQ), dtype=int)
            obs.Pstd = np.zeros((nsat, gn.MAX_NFREQ), dtype=int)
            obs.Lstd = np.zeros((nsat, gn.MAX_NFREQ), dtype=int)
            obs.mag = np.zeros((nsat, gn.MAX_NFREQ))
            obs.sat = np.zeros(nsat, dtype=int)
            n = 0
            for k in range(nsat):
                line = self.fobs.readline()
                if line[0] not in self.gnss_tbl:
                    continue
                sys = self.gnss_tbl[line[0]]
                if sys not in nav.gnss_t:
                    continue
                prn = int(line[1:3])
                if sys == uGNSS.QZS:
                    prn += 192
                obs.sat[n] = prn2sat(sys, prn)
                if obs.sat[n] == 0:
                    continue
                nsig_max = (len(line) - 4 + 2) // 16
                for i in range(self.nsig[sys]):
                    if i >= nsig_max:
                        break
                    band = self.sigband[sys][i]
                    # Mapping keys are canonical RINEX system characters;
                    # ``sys`` is the decoder's uGNSS enum.
                    band_slot = self.raw_band_to_slot.get(line[0], {})
                    if band <= 0 or band not in band_slot:
                        # Unknown signal codes carry no signal id.  They are
                        # harmless distractors when outside this stream's
                        # target bands, but known unsupported bands retain the
                        # historical hard failure and identity.
                        if self.sigid[sys][i] == 0:
                            continue
                        # Do not silently assign an unsupported raw band by
                        # column position: that can overwrite another band
                        # and expose a false dual-frequency observation.
                        sys_name = {
                            uGNSS.GPS: "GPS",
                            uGNSS.GLO: "GLO",
                            uGNSS.GAL: "GAL",
                            uGNSS.BDS: "BDS",
                            uGNSS.QZS: "QZS",
                        }.get(sys, str(sys))
                        raise SystemExit(
                            f"{sys_name} raw band {band} ({line[0]}{band}) "
                            "is unsupported; simplify the RINEX observations"
                        )
                    obs_ = line[16*i+4:16*i+17].strip()
                    if obs_ == '':
                        continue
                    if self.sigid[sys][i] == 0:
                        sys_name = {
                            uGNSS.GPS: "GPS",
                            uGNSS.GLO: "GLO",
                            uGNSS.GAL: "GAL",
                            uGNSS.BDS: "BDS",
                            uGNSS.QZS: "QZS",
                        }.get(sys, str(sys))
                        raise SystemExit(
                            f"{sys_name} ({line[0]}) observation code for raw band {band} "
                            f"({line[0]}{band}) has no signal mapping"
                        )
                    try:
                        obsval = float(obs_)
                    except:
                        obsval = 0
                    f = band_slot[band]
                    if f >= gn.MAX_NFREQ:
                        sys_name = {
                            uGNSS.GPS: "GPS",
                            uGNSS.GLO: "GLO",
                            uGNSS.GAL: "GAL",
                            uGNSS.BDS: "BDS",
                            uGNSS.QZS: "QZS",
                        }.get(sys, str(sys))
                        raise SystemExit(
                            f"{sys_name} raw band {band} ({line[0]}{band}) "
                            f"maps to decoder slot {f}, beyond MAX_NFREQ"
                        )
                    if self.typeid[sys][i] == 0:  # code
                        obs.P[n, f] = obsval
                        Pstd = line[16*i+18]
                        obs.Pstd[n, f] = int(Pstd) if Pstd != " " else 0
                    elif self.typeid[sys][i] == 1:  # carrier
                        obs.L[n, f] = float(obs_)
                        lli = line[16*i+17]
                        obs.lli[n, f] = int(lli) if lli != " " else 0
                        Lstd = line[16*i+18]
                        obs.Lstd[n, f] = int(Lstd) if Lstd != " " else 0
                    elif self.typeid[sys][i] == 2:  # C/No
                        obs.S[n, f] = obsval
                    elif self.typeid[sys][i] == 3:  # Doppler
                            obs.D[n, f] = obsval
                n += 1
            obs.P = obs.P[:n, :]
            obs.L = obs.L[:n, :]
            obs.Pstd = obs.Pstd[:n, :]
            obs.Lstd = obs.Lstd[:n, :]
            obs.D = obs.D[:n, :]
            obs.S = obs.S[:n, :]
            obs.lli = obs.lli[:n, :]
            obs.mag = obs.mag[:n, :]
            obs.sat = obs.sat[:n]
            self.obslist.append(obs)
            nepoch += 1
            if maxepoch != None and nepoch >= maxepoch:
                break
        self.index = 0
        self.fobs.close()

    
    def decode_obsfile(self, nav, obsfile, maxepoch):
        self.decode_obsh(obsfile)
        self.decode_obs(nav, maxepoch)

def first_obs(nav, rov, base, dir):
    if dir == 1: # forward solution
        rov.index = base.index = 0
    else: # backward solution
        rov.index = len(rov.obslist) - 1
        base.index = len(base.obslist) - 1
    # sync base and rover, step one obs to sync
    _, _ =next_obs(nav, rov, base, dir)
    # step back to first obs
    obsr, obsb = next_obs(nav, rov, base, -dir)
    return obsr, obsb

def next_obs(nav, rov, base, dir):
    """ sync observations between rover and base """
    rov.index += dir   # 1=forward, -1=backward
    if abs(dir) != 1 or rov.index < 0 or rov.index >= len(rov.obslist):
        return [], []
    obsr, obsb = rov.obslist[rov.index], base.obslist[base.index]
    dt = timediff(obsr.t, obsb.t)
    baseChange = False
    ixb = base.index + dir
    while True:
        if ixb < 0 or ixb >= len(base.obslist):
            ixb -= dir
            dt_next = dt
            break # hit end of obs list
        dt_next = timediff(obsr.t, base.obslist[ixb].t)
        if abs(dt_next) >= abs(dt):
            break # next base obs is not closer
        else:
            base.index = ixb
            ixb += dir
            baseChange = True
            dt = dt_next
        
    if baseChange and nav.interp_base and len(nav.sol) > 0:
        # save base residuals for next epoch
        nav.obsb = deepcopy(obsb)
        nav.rsb, nav.varb, nav.dtsb, nav.svhb = satposs(obsb, nav)
    obsb = base.obslist[base.index]
    return obsr, obsb

def rcvstds(nav, obs):
    """ decode receiver stdevs from rinex fields """
    # skip if weighting factor is zero
    if nav.err[5] == 0:
        return
    for i in np.argsort(obs.sat):
        for f in range(nav.nf):
            s = obs.sat[i] - 1
            # decode receiver stdevs, 
            # Lstd: 0.004 cycles -> m
            nav.rcvstd[s,f] = obs.Lstd[i,f] * 0.004 * 0.2
            # Pstd: 0.01*2^(n+5)
            nav.rcvstd[s,f+nav.nf] = 0.01 * (1 << (obs.Pstd[i,f] + 5))
