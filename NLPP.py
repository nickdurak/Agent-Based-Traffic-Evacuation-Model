import getopt, sys

class ProcessingError(Exception):
	def __init__(self, value):
		self.value = value
	def __str__(self):
		return repr(self.value)


def hasDelim(s):
	"Check if s has the delimiter ';; $@'"
	return s.strip().startswith(";; $@")

def getDelimName(s):
	"Get the name of this delimiter"
	return s.strip()[5:]
	
def getConfigTuple(s):
	"Get the name of this config option and its value"
	name, space, value = s.strip()[5:].partition(" ")
	if ( name != "VERSION" ):
		value = (value == "TRUE")
	return (name, value)

def endOfCode(s):
	"Check to see if we are at the end of the code"
	return s.startswith("@#$#@#$#@")

def commentCode(line):
	"Comment a line of code that is not already commented out"
	if ( line.strip()[0:1] == ";" and line.strip()[1:2] != ";" ):
		# skip lines that are already commented out, but not comments (;;)
		return line
	else:
		return "; " + line

def uncommentCode(line):
	"Uncomment a line of code"
	if ( line.strip()[0:2] == ";;" or line.strip()[0:1] != ";" ):
		return line
	else:
		semiPos = line.find(";")
		return line[0:semiPos] + line[semiPos + 1:]

def processFile(inFileName, verbose = True, outFileName = None):
	"Process a file line by line"
	#file = open(fileName, 'r')
	
	# open a default output file
	if outFileName == None:
		filenameExtPos = inFileName.rfind(".")
		outFileName = inFileName[0:filenameExtPos] + ".post" + inFileName[filenameExtPos:]
	output = open(outFileName, 'wb')
	
	#state tells us where we are in processing
	STATE_NORMAL = 0
	STATE_CONFIG = 1
	STATE_COMMENT = 2
	STATE_UNCOMMENT = 3
	STATE_ENDOFCODE = 4
	state = 0
	
	#keep track of types to comment/uncomment
	config = {}
	
	with open(inFileName, 'r') as file:
		for line in file:
			lineOut = line
			if ( endOfCode(line) ):
				state = STATE_ENDOFCODE
				if verbose:
					print "End of processing"
			elif ( hasDelim(line) and getDelimName(line) == "END" ):
				state = STATE_NORMAL
			elif ( state == STATE_CONFIG ):
				name, value = getConfigTuple(line)
				config[name] = value
				if verbose:
					print "Read config pair (", name, "," , value, ")"
			elif ( state == STATE_NORMAL and hasDelim(line) ):
				name = getDelimName(line)
				if verbose:
					print name
				if ( name == "CONFIG" ):
					state = STATE_CONFIG
					if verbose:
						print "processing config"
				elif ( name in config ):
					if ( config[name] ):
						state = STATE_UNCOMMENT
						if verbose:
							print "uncommenting..."
					else:
						state = STATE_COMMENT
						if verbose:
							print "commenting..."
			elif ( state == STATE_COMMENT ):
				if ( hasDelim(line) and getDelimName(line) == "END" ):
					state = STATE_NORMAL
				else:
					if ( hasDelim(line) ):
						raise ProcessingError("***Unexpected preprocessor tag found")
					lineOut = commentCode(line)
			elif (state == STATE_UNCOMMENT ):
				if ( hasDelim(line) and getDelimName(line) == "END" ):
					state = STATE_NORMAL
				else:
					if ( hasDelim(line) ):
						raise ProcessingError("***Unexpected preprocessor tag found")
					lineOut = uncommentCode(line)
			output.write(lineOut)

def main():
	try:
		# -i file_name : input file
		# -o file_name : output file
		## -t : test only do not save
		# -s : silent, no output just save file
		opts, args = getopt.getopt(sys.argv[1:], "i:o:s")
	except getopt.GetoptError, err:
		# print help information and exit:
		print str(err) # will print something like "option -a not recognized"
		sys.exit(2)
	input = None
	output = None
	verbose = True
	for o, a in opts:
		if o == "-s":
			verbose = False
		elif o == "-o":
			output = a
		elif o == "-i":
			input = a
		else:
			assert False, "unhandled option"
	if (input == None or output == None):
		assert False, "Missing parameters"
	processFile( input, verbose, output )

if __name__ == "__main__":
	main()
