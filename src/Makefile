#################################################
#												#
#					Makefile	 				#
#												#
#################################################


## DEFAULT ACTION:
############################
default: doc


## HELP:
############################
help:
	@( \
	echo ;\
	echo "    === Makefile for the Autonomous Sail-boat ===                   ";\
	echo ;\
	echo "make        : Default action...                                     ";\
	echo "              Actually generates the documentation!                 ";\
	echo ;\
	echo "make backup : Generates a backup copy ...                           ";\
	echo ;\
	echo "make doc    : Cleans then generates the documentation (doxygen)     ";\
	echo "              which is located: '../doc/Sailboat_Documentation.html'";\
	echo ;\
	echo "make clean  : Cleans temporary files (including the                 ";\
	echo "              actual documentation)                                 ";\
	echo "";\
	)


## BACKUP:
############################
# Save the existing document by compressing it
backup:
	7z a ../../"Backup_"$(shell date '+%Y-%m-%d') ../../AutonomousSailboatPlymouth
	@( \
	echo ;\
	echo "############################################";\
	echo "#  === Backup copy completed ===           #";\
	echo "#                                          #";\
	echo "#  See ../../Backup*.7z                    #";\
	echo "############################################";\
	)


## DOCUMENTATION:
############################
doc: clean
	doxygen AutonomousSailboat.doxyfile
	ln -rsv ../doc/content/files.html ../doc/Sailboat_Documentation.html
	@( \
	echo ;\
	echo "############################################";\
	echo "#  === Documentation completed ===         #";\
	echo "#                                          #";\
	echo "#  See ../doc/Sailboat_Documentation.html  #";\
	echo "############################################";\
	)


## CLEANING:
############################
clean:
	rm -rf ./../doc/Sailboat_Documentation.html
	rm -rf ./../doc/content/*