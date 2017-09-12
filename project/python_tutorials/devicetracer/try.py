msg = ""
result = [0x12, 0x34, 0x56, 0x78, 0x89, 0x9A, 0xAB, 0xBC]
for index in range(8):
    #data_tuple = data_tuple + ("%.2X" % result.DATA[index],)
    msg = "%s %.2X" % (msg, result[index])
    strom_index = 21/8  # begin 21, len 14
    #((0x1F&0b111)<<11) + ((0xFF&0xFC)<<3) + ((0b11100000 & 0x8D )>>5)
    if index == strom_index:
        strom_wert_teil1 = (0b111 & result[index])<<11
    if index == (strom_index+1):
        strom_wert_teil2 = (0xFF & result[index])<<3
    if index == (strom_index+2):
        strom_wert_teil3 = (0b11100000 & result[index])>>5
        strom_total = (strom_wert_teil1 + strom_wert_teil2 + strom_wert_teil3)/10
        strom_total_factor = strom_total/10 - 819.5
print result
print strom_total
print strom_total_factor
