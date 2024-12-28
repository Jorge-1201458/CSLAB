# Variables 
INCLUDES = -I/usr/local/include  # Use the path you found for the header files
LDFLAGS = -L/usr/local/lib      # Path to the libraries
LIBS = -lpaho-mqtt3cs           # Link with the Paho MQTT client library

SOURCES = server.c   
OBJFILES = server.o   
EXEC = prog.bin 

# Compilation parameters 
CP = -Wall -Wextra -pthread  # Added -pthread for thread support

# Rules for suffixes 
.SUFFIXES: .c .o  

# Transform .c into .o ; $< -- filename 
.c.o:  
	gcc -g ${CP} ${INCLUDES} -c $<    

# Main program 
${EXEC}: ${OBJFILES}  
	gcc -g ${CP} ${INCLUDES} ${LDFLAGS} -o ${EXEC} ${OBJFILES} ${LIBS}  

# Run the program 
run: ${EXEC}  
	./${EXEC}  

# Clean up object files and executable 
clean:
	rm -f ${OBJFILES} ${EXEC}
