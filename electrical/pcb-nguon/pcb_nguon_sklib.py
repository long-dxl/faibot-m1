from collections import defaultdict
from skidl import Pin, Part, Alias, SchLib, SKIDL, TEMPLATE

from skidl.pin import pin_types

SKIDL_lib_version = '0.0.1'

pcb_nguon = SchLib(tool=SKIDL).add_parts(*[
        Part(**{ 'name':'BOLT_M6_2P', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'BOLT_M6_2P'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'Terminal_M6_2P', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='POS',func=pin_types.PWRIN),
            Pin(num='2',name='NEG',func=pin_types.PWRIN)] }),
        Part(**{ 'name':'FUSE', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'FUSE'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'Fuse_Holder', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='A',func=pin_types.PASSIVE),
            Pin(num='2',name='B',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'TVS', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'TVS'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'D_SMC', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='K',func=pin_types.PASSIVE),
            Pin(num='2',name='A',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'K1_PWR', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'K1_PWR'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'Terminal_M6_2P', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='K1_IN',func=pin_types.PASSIVE),
            Pin(num='2',name='K1_OUT',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'RELAY_SPST', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'RELAY_SPST'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'Relay_SPST_G2R', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='COIL_P',func=pin_types.PASSIVE),
            Pin(num='2',name='COIL_N',func=pin_types.PASSIVE),
            Pin(num='3',name='COM',func=pin_types.PASSIVE),
            Pin(num='4',name='NO',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'R_PWR', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'R_PWR'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'R_Power_5W', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='1',func=pin_types.PASSIVE),
            Pin(num='2',name='2',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'FUSE_PTC', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'FUSE_PTC'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'Fuse_1812', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='A',func=pin_types.PASSIVE),
            Pin(num='2',name='B',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'ESTOP_2NC', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'ESTOP_2NC'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'TerminalBlock_4P_5.08', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='NC1_A',func=pin_types.PASSIVE),
            Pin(num='2',name='NC1_B',func=pin_types.PASSIVE),
            Pin(num='3',name='NC2_A',func=pin_types.PASSIVE),
            Pin(num='4',name='NC2_B',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'BTN_1NO1NC', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'BTN_1NO1NC'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'TerminalBlock_4P_5.08', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='NO_A',func=pin_types.PASSIVE),
            Pin(num='2',name='NO_B',func=pin_types.PASSIVE),
            Pin(num='3',name='NC_A',func=pin_types.PASSIVE),
            Pin(num='4',name='NC_B',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'D', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'D'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'D_SMA', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='K',func=pin_types.PASSIVE),
            Pin(num='2',name='A',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'CP', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'CP'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'CP_Radial', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='+',func=pin_types.PASSIVE),
            Pin(num='2',name='-',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'RELAY_DPDT', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'RELAY_DPDT'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'Relay_DPDT_G2R', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='COIL_P',func=pin_types.PASSIVE),
            Pin(num='2',name='COIL_N',func=pin_types.PASSIVE),
            Pin(num='3',name='COM1',func=pin_types.PASSIVE),
            Pin(num='4',name='NO1',func=pin_types.PASSIVE),
            Pin(num='5',name='COM2',func=pin_types.PASSIVE),
            Pin(num='6',name='NO2',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'ZENER', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'ZENER'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'D_SOD-123', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='K',func=pin_types.PASSIVE),
            Pin(num='2',name='A',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'SCREW2', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'SCREW2'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'TerminalBlock_2P_5.08', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='P1',func=pin_types.PASSIVE),
            Pin(num='2',name='P2',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'NMOS', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'NMOS'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'SOT-23', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='G',func=pin_types.INPUT),
            Pin(num='2',name='S',func=pin_types.PASSIVE),
            Pin(num='3',name='D',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'R', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'R'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'R_0603', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='1',func=pin_types.PASSIVE),
            Pin(num='2',name='2',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'C', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'C'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'C_0805', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='1',func=pin_types.PASSIVE),
            Pin(num='2',name='2',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'74LVC1G17', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'74LVC1G17'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'SOT-23-5', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='2',name='A',func=pin_types.INPUT),
            Pin(num='3',name='GND',func=pin_types.PWRIN),
            Pin(num='4',name='Y',func=pin_types.OUTPUT),
            Pin(num='5',name='VCC',func=pin_types.PWRIN)] }),
        Part(**{ 'name':'74AHCT1G125', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'74AHCT1G125'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'SOT-23-5', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='OE_N',func=pin_types.INPUT),
            Pin(num='2',name='A',func=pin_types.INPUT),
            Pin(num='3',name='GND',func=pin_types.PWRIN),
            Pin(num='4',name='Y',func=pin_types.OUTPUT),
            Pin(num='5',name='VCC',func=pin_types.PWRIN)] }),
        Part(**{ 'name':'74LVC1G74', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'74LVC1G74'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'VSSOP-8', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='CLK',func=pin_types.INPUT),
            Pin(num='2',name='D',func=pin_types.INPUT),
            Pin(num='3',name='QN',func=pin_types.OUTPUT),
            Pin(num='4',name='GND',func=pin_types.PWRIN),
            Pin(num='5',name='Q',func=pin_types.OUTPUT),
            Pin(num='6',name='CLR_N',func=pin_types.INPUT),
            Pin(num='7',name='PRE_N',func=pin_types.INPUT),
            Pin(num='8',name='VCC',func=pin_types.PWRIN)] }),
        Part(**{ 'name':'74LVC1G08', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'74LVC1G08'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'SOT-23-5', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='A',func=pin_types.INPUT),
            Pin(num='2',name='B',func=pin_types.INPUT),
            Pin(num='3',name='GND',func=pin_types.PWRIN),
            Pin(num='4',name='Y',func=pin_types.OUTPUT),
            Pin(num='5',name='VCC',func=pin_types.PWRIN)] }),
        Part(**{ 'name':'SHUNT4', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'SHUNT4'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'Shunt_4T_2512', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='F_IN',func=pin_types.PASSIVE),
            Pin(num='2',name='F_OUT',func=pin_types.PASSIVE),
            Pin(num='3',name='S_P',func=pin_types.PASSIVE),
            Pin(num='4',name='S_N',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'INA226', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'INA226'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'VSSOP-10', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='A1',func=pin_types.INPUT),
            Pin(num='2',name='A0',func=pin_types.INPUT),
            Pin(num='3',name='ALERT',func=pin_types.OPENCOLL),
            Pin(num='4',name='SDA',func=pin_types.BIDIR),
            Pin(num='5',name='SCL',func=pin_types.INPUT),
            Pin(num='6',name='VS',func=pin_types.PWRIN),
            Pin(num='7',name='GND',func=pin_types.PWRIN),
            Pin(num='8',name='VBUS',func=pin_types.INPUT),
            Pin(num='9',name='IN-',func=pin_types.INPUT),
            Pin(num='10',name='IN+',func=pin_types.INPUT)] }),
        Part(**{ 'name':'BOLT_M6_ARM', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'BOLT_M6_ARM'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'Terminal_M6_2P', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='POS',func=pin_types.PASSIVE),
            Pin(num='2',name='NEG',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'XT60PW', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'XT60PW'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'XT60PW-M', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='POS',func=pin_types.PASSIVE),
            Pin(num='2',name='NEG',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'TPS54560B', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'TPS54560B'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'HSOIC-8_PowerPAD', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='BOOT',func=pin_types.PASSIVE),
            Pin(num='2',name='VIN',func=pin_types.PWRIN),
            Pin(num='3',name='EN',func=pin_types.INPUT),
            Pin(num='4',name='RT_CLK',func=pin_types.PASSIVE),
            Pin(num='5',name='FB',func=pin_types.PASSIVE),
            Pin(num='6',name='COMP',func=pin_types.PASSIVE),
            Pin(num='7',name='GND',func=pin_types.PWRIN),
            Pin(num='8',name='SW',func=pin_types.PASSIVE),
            Pin(num='9',name='PAD',func=pin_types.PWRIN)] }),
        Part(**{ 'name':'L', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'L'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'L_Bourns_SRP1245', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='1',func=pin_types.PASSIVE),
            Pin(num='2',name='2',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'XT30PW', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'XT30PW'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'XT30PW-M', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='POS',func=pin_types.PASSIVE),
            Pin(num='2',name='NEG',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'LED', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'LED'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'LED_0805', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='K',func=pin_types.PASSIVE),
            Pin(num='2',name='A',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'HDR_PI_CTRL', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'HDR_PI_CTRL'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'PinHeader_1x09', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='P1',func=pin_types.PASSIVE),
            Pin(num='2',name='P2',func=pin_types.PASSIVE),
            Pin(num='3',name='P3',func=pin_types.PASSIVE),
            Pin(num='4',name='P4',func=pin_types.PASSIVE),
            Pin(num='5',name='P5',func=pin_types.PASSIVE),
            Pin(num='6',name='P6',func=pin_types.PASSIVE),
            Pin(num='7',name='P7',func=pin_types.PASSIVE),
            Pin(num='8',name='P8',func=pin_types.PASSIVE),
            Pin(num='9',name='P9',func=pin_types.PASSIVE)] }),
        Part(**{ 'name':'HDR_ALERT', 'dest':TEMPLATE, 'tool':SKIDL, 'aliases':Alias({'HDR_ALERT'}), 'ref_prefix':'U', 'fplist':None, 'footprint':'PinHeader_1x02', 'keywords':None, 'description':'', 'datasheet':None, 'pins':[
            Pin(num='1',name='P1',func=pin_types.PASSIVE),
            Pin(num='2',name='P2',func=pin_types.PASSIVE)] })])