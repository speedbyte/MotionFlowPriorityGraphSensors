""" 


    $Date: 2008-12-19 10:21:31 +0100 (Fr, 19 Dez 2008) $
    $Rev: 3751 $
    $Author: agrawal $
    $URL: svn://voice/work/VA/Python_work/easyexcel.py $


"""

_svn_id = "$Id$"
_svn_rev = "$Rev: 3751 $"
__version__ = _svn_rev[6:-2]

import win32com.client

borderTop = 3
borderBottom = 4
borderLeft = 1
borderRight = 2
borderSolid = 1
borderDashed = 2
borderDotted = 3
colorBlack = 1
directionUp = -4162
directionDown = -4121
directionLeft = -4131
directionRight = -4152

class easyExcel:
    """A utility to make it easier to get at Excel. Remembering
    to save the data is your problem, as is error handling.
    Operates on one workbook at a time."""
    def __init__(self, filename=None, visible=False, sheet=1):
        self.xlApp = win32com.client.Dispatch('Excel.Application')
        self.xlApp.Visible = visible
        # Init with first sheet
        #self.sheet = self.xlApp.Sheets(sheet)
        #self.sheet = self.xlApp.Sheets(1)
        if filename:
            self.filename = filename
            self.xlBook = self.xlApp.Workbooks.Add(filename)
        # filename needed
        else:
            self.filename = '%s.xls' % ticker
            self.xlBook = self.xlApp.Workbooks.Add()

    def new(self, filename=None):
        """
        Create a new Excel workbook. If 'filename' specified,
        use the file as a template.
        """
        self.app.Workbooks.Add(filename)
    
    def clearSheet(self):
        for i in 30:
            delete_row(i)
        for j in 30:
            delete_column(j)
    
    def int2letters(zahl):       
        return ("%s%s"%(' ABCDEFGHIJKLMNOPQRSTUVWXYZ'[(zahl-1)/26],'ABCDEFGHIJKLMNOPQRSTUVWXYZ'[(zahl % 26)-1])).strip()
        
    def letters2int(str):
        return ((ord(str[-2]) - ord('A') + 1) * 26) + ord(str[-1]) - ord('A') + 1

    def setSheet(self, sheet):
        """
        Set the active worksheet.
        """
        self.sheet = self.xlApp.Sheets(sheet)
        
    def save(self, newfilename=None):
        if newfilename:
            self.filename = newfilename
            self.xlBook.SaveAs(newfilename)
        else:
            self.xlBook.Save()
            
    def close(self):
        self.xlBook.Close(SaveChanges=0)
        del self.xlApp

    def getValue(self, row, col):
        "Get value of one cell"
        return self.sheet.Cells(row, col).Value

    def setValue(self, cell, value):
        self.sheet.getRange(cell).Value = value

    def setValue(self, row, col, value):
        "set value of one cell"
        self.sheet.Cells(row,col).Value = value

    def getRange(self, range):
        """
        Get a range object for the specified range or single cell.
        """
        return self.sheet.Range(range)

    def getSpecificRange(self, row1, col1, row2, col2):
        "return a 2d array (i.e. tuple of tuples)"
        return self.sheet.Range(self.sheet.Cells(row1, col1), self.sheet.Cells(row2, col2)).Value

    def getContiguousRange(self, row, col):
        """Tracks down and across from top left cell until it
        encounters blank cells; returns the non-blank range.
        Looks at first row and column; blanks at bottom or right
        are OK and return None within the array"""

        # find the bottom row
        bottom = row
        while sheet.Cells(bottom + 1, col).Value not in [None, ""]:
            bottom = bottom + 1

        # right column
        right = col
        while sheet.Cells(row, right + 1).Value not in [None, ""]:
            right = right + 1

        return sheet.Range(sheet.Cells(row, col), sht.Cells(bottom, right)).Value

    def delete_row(self, row, shift=directionUp):
        """
        Delete the entire 'row'.
        """
        self.getRangeObject('a%s' % row).EntireRow.Delete(Shift=shift)

    def delete_column(self, col, shift=directionLeft):
        """
        Delete the entire 'column'.
        """
        self.getRangeObject('1%s' % col).EntireColumn.Delete(Shift=shift)
        
    def quit(self):
        """
        Quit Excel.
        """
        return self.app.Quit()
    def fit_column(self, column):
        """
        Resize the specified 'column' to fit all its contents.
        """
        self.getRange('1%s' % column).EntireColumn.AutoFit()
        
    def fixStringsAndDates(self, aMatrix):
        # converts all unicode strings and times
        newmatrix = []
        for row in aMatrix:
            newrow = []
            for cell in row:
                if type(cell) is UnicodeType:
                    newrow.append(str(cell))
                elif type(cell) is TimeType:
                    newrow.append(int(cell))
                else:
                    newrow.append(cell)
            newmatrix.append(tuple(newrow))
        return newmatrix
