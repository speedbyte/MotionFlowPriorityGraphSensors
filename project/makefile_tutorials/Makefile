#target:dependancies
#   command

#target:prerequisites
#   recipe

TARGET=IOVEC-DEMO

$(TARGET) : iovec-demo.o 
	gcc -o exec-dir/$@ object-dir/$<

#PHONY : object-dir/iovec-demo.o
iovec-demo.o : %.o : %.c
	gcc -c -o object-dir/$@ $<

all : $(TARGET)
#IOVEC-DEMO: FORCE
#ORCE:

clean:
	rm -rf object-dir/*.o
