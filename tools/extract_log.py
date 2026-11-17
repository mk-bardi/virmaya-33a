"""Extract key message types from an ArduPilot .BIN into CSVs. Usage: extract_log.py <bin> <outdir>"""
import sys, csv, os
from pymavlink import mavutil

TYPES = {
    'ATT': ['TimeUS', 'DesRoll', 'Roll', 'DesPitch', 'Pitch', 'DesYaw', 'Yaw'],
    'RATE': ['TimeUS', 'RDes', 'R', 'ROut', 'PDes', 'P', 'POut', 'YDes', 'Y', 'YOut', 'ADes', 'A', 'AOut'],
    'RCOU': ['TimeUS', 'C1', 'C2', 'C3', 'C4', 'C5', 'C6', 'C7', 'C8'],
    'RCIN': ['TimeUS', 'C1', 'C2', 'C3', 'C4'],
    'GPS': ['TimeUS', 'I', 'Status', 'Lat', 'Lng', 'Alt', 'Spd', 'GCrs', 'VZ', 'Yaw'],
    'XKF1': ['TimeUS', 'C', 'Roll', 'Pitch', 'Yaw', 'VN', 'VE', 'VD', 'PN', 'PE', 'PD'],
    'XKF3': ['TimeUS', 'C', 'IVN', 'IVE', 'IVD', 'IPN', 'IPE', 'IPD', 'IMX', 'IMY', 'IMZ', 'IYAW'],
    'CTUN': ['TimeUS','NavRoll','Roll','NavPitch','Pitch','ThO','RdrOut','ThD','As','SAs','E2T','GU'],
    'TECS': ['TimeUS','h','dh','hdem','dhdem','spdem','sp','dsp','th','ph'],
    'XKF4': ['TimeUS', 'C', 'SV', 'SP', 'SH', 'SM', 'SVT', 'FS', 'TS', 'SS', 'GPS', 'PI'],
    'MODE': ['TimeUS', 'Mode', 'ModeNum', 'Rsn'],
    'MSG': ['TimeUS', 'Message'],
    'QTUN': ['TimeUS', 'ThI', 'ABst', 'ThO', 'ThH', 'DAlt', 'Alt', 'BAlt', 'DCRt', 'CRt', 'Trn', 'Ast', 'TMix', 'Sscl'],
    'BAT': ['TimeUS', 'Inst', 'Volt', 'Curr', 'RemPct'],
    'SIM': ['TimeUS', 'Roll', 'Pitch', 'Yaw', 'Alt', 'Lat', 'Lng'],
    'POS': ['TimeUS', 'Lat', 'Lng', 'Alt', 'RelHomeAlt', 'RelOriginAlt'],
    'PSCN': ['TimeUS', 'TPN', 'PN', 'DVN', 'TVN', 'VN', 'DAN', 'TAN', 'AN'],
    'PSCE': ['TimeUS', 'TPE', 'PE', 'DVE', 'TVE', 'VE', 'DAE', 'TAE', 'AE'],
    'MAG': ['TimeUS', 'I', 'MagX', 'MagY', 'MagZ'],
    'EV': ['TimeUS', 'Id'],
}
mlog = mavutil.mavlink_connection(sys.argv[1])
out = sys.argv[2]
os.makedirs(out, exist_ok=True)
files, writers, fields = {}, {}, {}
while True:
    m = mlog.recv_match(type=list(TYPES))
    if m is None:
        break
    t = m.get_type()
    if t not in writers:
        avail = [f for f in TYPES[t] if f in m.get_fieldnames()]
        fields[t] = avail
        files[t] = open(f'{out}/{t}.csv', 'w', newline='')
        writers[t] = csv.writer(files[t])
        writers[t].writerow(avail)
    writers[t].writerow([getattr(m, f) for f in fields[t]])
for f in files.values():
    f.close()
print({t: fields[t] for t in fields})
