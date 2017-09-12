import _mysql
from _mysql_exceptions import OperationalError
import  time


class ZigbeeSqlConnector(object):
    def __init__(self):
        self.database_connection_established = False
    
    def connect_database(self):
        try:
            db =_mysql.connect()
        except OperationalError as e:
            if ( e[0] == 1045):
                print "Access is Denied"      
            print e
        else:
            print db
        finally:
            try:
                db= _mysql.connect("localhost","root","asdf","niharika_db")
            except OperationalError as e:
                print "niharika doesnt exist ", e
                self.database_connection_established = False
            else:
                self.database_connection_established = True
        return db
            
    def queries(self, db):
        if ( self.database_connection_established == True ):
            print db
            db.execute("SELECT * from niharika_icurb where ps=0")
            #db.execute ("INSERT INTO `niharika_db`.`niharika_icurb` (`ps`, `datetime`, `mac`) VALUES ('3', '14:01', '00:aa');")
        
    
    def readZigbee(self):
        fd = open("niharika_zigbee.txt", 'r')
        for line in fd.readlines():
            print line
            splitted_line = line.split(";")
        return splitted_line

    def close(self,db):
        db.close()


if __name__ == "__main__":
    # Test the module
    try:
        instanceZigbeeSqlConnector = ZigbeeSqlConnector()
        database = instanceZigbeeSqlConnector.connect_database()
        timestring = time.asctime( time.localtime(time.time()) )
        print timestring
        splitted_line = instanceZigbeeSqlConnector.readZigbee()
        instanceZigbeeSqlConnector.queries(database)
    finally:
        database.close()