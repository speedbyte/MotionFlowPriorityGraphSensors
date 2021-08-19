from pyPdf import PdfFileWriter, PdfFileReader

output = PdfFileWriter()
input1 = PdfFileReader(file("EV_UHVNA_S02010_anw_de.pdf", "rb"))

# print the title of document1.pdf
print "title = %s" % (input1.getDocumentInfo().title)

output.addPage(input1.getPage(1))
output.addPage(input1.getPage(2))
output.addPage(input1.getPage(3))
output.addPage(input1.getPage(3))

# print how many pages input1 has:
print "document1.pdf has %s pages." % input1.getNumPages()

# finally, write "output" to document-output.pdf
outputStream = file("document-output.pdf", "wb")

output.write(outputStream)
outputStream.close()

