import re
import subprocess

device_re = re.compile("Bus\s+(?P<bus>\d+)\s+Device\s+(?P<device>\d+).+ID\s(?P<id>\w+:\w+)\s(?P<tag>.+)$", re.I)
df = subprocess.check_output("lsusb", encoding="utf-8")
devices = []
for i in df.split('\n'):
    if i:
        info = device_re.match(i)
        if info:
            dinfo = info.groupdict()
            dinfo['device'] = '/dev/bus/usb/%s/%s' % (dinfo.pop('bus'), dinfo.pop('device'))
            devices.append(dinfo)

can_re = re.compile("can", re.I)
no_file_re = re.compile("No such file or directory", re.I)

acm_dev = subprocess.getoutput("ls /dev/ttyA*")
usb_dev = subprocess.getoutput("ls /dev/ttyU*")

for device in devices:
    ids, tag, port = device.values()
    res = can_re.search(tag)
    if res is not None:
        print(f"Company: {tag}")
        print(f"Absolute port: {port}")
        if no_file_re.search(usb_dev) is None:
            print(f"USB port: {usb_dev}")
        if no_file_re.search(acm_dev) is None:
            print(f"ACM port: {acm_dev}")
