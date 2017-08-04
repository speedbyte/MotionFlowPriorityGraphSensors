import re
p = re.compile(ur'(rm)|(fr)|n{1,}$|(er)\w*$|\b(fr)\w*|\A\w+', re.MULTILINE)
test_str = u"carmenschaumann\ncarmenschaumannweikasfreimeister\nfreimeister\nfrei frsiter"
 
found = re.findall(p, test_str)
print found
