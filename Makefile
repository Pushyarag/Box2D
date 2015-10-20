.SUFFIXES: .cpp .hpp

# Programs
SHELL 	= bash
CC     	= g++
LD	= ld
RM 	= rm
ECHO	= /bin/echo
CAT	= cat
PRINTF	= printf
SED	= sed
DOXYGEN = doxygen
######################################
# Project Name (generate executable with this name)
TARGET = cs251_base

# Project Paths
PROJECT_ROOT=./
EXTERNAL_ROOT=$(PROJECT_ROOT)/external
SRCDIR = $(PROJECT_ROOT)/src
OBJDIR = $(PROJECT_ROOT)/obj
BINDIR = $(PROJECT_ROOT)/bin
DOCDIR = $(PROJECT_ROOT)/doc

# Library Paths
BOX2D_ROOT=$(EXTERNAL_ROOT)
GLUI_ROOT=/usr
GL_ROOT=/usr/include/

#Libraries
LIBS = -lBox2D -lglui -lglut -lGLU -lGL

# Compiler and Linker flags
CPPFLAGS =  -O3 -Wall -fno-strict-aliasing
CPPFLAGS+=-I $(BOX2D_ROOT)/include -I $(GLUI_ROOT)/include
LDFLAGS+=-L $(BOX2D_ROOT)/lib -L $(GLUI_ROOT)/lib




######################################

NO_COLOR=\e[0m
OK_COLOR=\e[1;32m
ERR_COLOR=\e[1;31m
WARN_COLOR=\e[1;33m
MESG_COLOR=\e[1;34m
FILE_COLOR=\e[1;37m

OK_STRING="[OK]"
ERR_STRING="[ERRORS]"
WARN_STRING="[WARNINGS]"
OK_FMT="${OK_COLOR}%30s\n${NO_COLOR}"
ERR_FMT="${ERR_COLOR}%30s\n${NO_COLOR}"
WARN_FMT="${WARN_COLOR}%30s\n${NO_COLOR}"
######################################

SRCS := $(wildcard $(SRCDIR)/*.cpp)
INCS := $(wildcard $(SRCDIR)/*.hpp)
OBJS := $(SRCS:$(SRCDIR)/%.cpp=$(OBJDIR)/%.o)


.PHONY: all setup codeDoc clean distclean report profile release
all: setup release codeDoc report 


setup:
	@$(ECHO) ""
	@$(ECHO) "Setting up compilation..."
	@mkdir -p obj
	@mkdir -p bin
	

$(BINDIR)/$(TARGET): $(OBJS)
	@$(PRINTF) "$(MESG_COLOR)Building executable:$(NO_COLOR) $(FILE_COLOR) %16s$(NO_COLOR)" "$(notdir $@)"
	@$(CC) -pg -o $@ $(LDFLAGS) -O3 $(OBJS) $(LIBS) 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else $(PRINTF) $(OK_FMT) $(OK_STRING); \
	fi;
	@$(RM) -f temp.log temp.err

-include -include $(OBJS:.o=.d)

$(OBJS): $(OBJDIR)/%.o : $(SRCDIR)/%.cpp
	@$(PRINTF) "$(MESG_COLOR)Compiling: $(NO_COLOR) $(FILE_COLOR) %25s$(NO_COLOR)" "$(notdir $<)"
	@$(CC) $(CPPFLAGS) -pg -c $< -o $@ -MD 2> temp.log || touch temp.err
	@if test -e temp.err; \
	then $(PRINTF) $(ERR_FMT) $(ERR_STRING) && $(CAT) temp.log; \
	elif test -s temp.log; \
	then $(PRINTF) $(WARN_FMT) $(WARN_STRING) && $(CAT) temp.log; \
	else printf "${OK_COLOR}%30s\n${NO_COLOR}" "[OK]"; \
	fi;
	@$(RM) -f temp.log temp.err

codeDoc:
	@$(ECHO) ""
	@$(ECHO) -n "Generating Doxygen Documentation ...  "
	@$(RM) -rf doc/html
	@$(DOXYGEN) $(DOCDIR)/Doxyfile 2 > /dev/null
	@$(ECHO) ""
	@$(ECHO) " Documenting Done"

clean:
	@$(ECHO) ""
	@$(ECHO) -n "Cleaning up..."
	@$(RM) -rf $(OBJDIR) *~ $(DEPS) $(SRCDIR)/*~
	@$(ECHO) ""
	@$(ECHO) " cleaning Done"

distclean: clean
	@$(RM) -rf $(BINDIR) $(DOCDIR)/html profile
	@$(ECHO) ""
	@$(ECHO) " directory clean Done"

	
profile: setup $(BINDIR)/$(TARGET)
	@mkdir -p profile
	@$(ECHO) ""
	@$(ECHO) "Profiling using gprof ..."
	@cd profile && ../bin/cs251_base 
	@cd profile && gprof ../bin/cs251_base gmon.out > analysis.txt
	@$(ECHO) ""
	@$(ECHO) "Generating dot file ..."
	@cd profile && python ../external/gprof2dot.py analysis.txt -o analysis.dot
	@$(ECHO) ""
	@$(ECHO) "Generating callgraph ..."
	@cd profile && dot -Tpng analysis.dot -o callgraph.png
	@$(ECHO) ""
	@$(ECHO) "Profiling Done"

release: setup 
	@pushd ./external/src/
	@cd ./external/src/ && tar xvzf Box2D.tgz
	@cd ./external/src/Box2D/ && mkdir -p build251
	@cd ./external/src/Box2D/build251/ && cmake -DCMAKE_BUILD_TYPE=Release ../
	@cd ./external/src/Box2D/build251/ && make
	@cd ./external/src/Box2D/build251/ && make install
	@make $(BINDIR)/$(TARGET)
	@$(ECHO) ""
	@$(ECHO) "Release version successfully created ..."
report: 
	@mkdir -p reports
	@$(ECHO) ""
	@$(ECHO) "Generating report.pdf ..."
	@$ pdflatex report.tex tex 1>/dev/null
	@$ bibtex report
	@$ pdflatex report.tex tex 1>/dev/null
	@$ bibtex report
	@$ pdflatex report.tex 1>/dev/null  
	@$ pdflatex Beamer.tex 1>/dev/null 
	@$(ECHO) ""
	@$(ECHO) "Generating slides.pdf ..."	
	@$ rm -rf report.aux || true	
	@$ rm -rf report.bbl || true
	@$ rm -rf report.log || true
	@$ rm -rf report.toc || true
	@$ rm -rf report.blg || true
	@$ rm -rf report.out || true
	@$ rm -rf report.lof || true
	@$ rm -rf Beamer.aux || true	
	@$ rm -rf Beamer.bbl || true
	@$ rm -rf Beamer.log || true
	@$ rm -rf Beamer.toc || true
	@$ rm -rf Beamer.blg || true
	@$ rm -rf Beamer.out || true
	@$ rm -rf Beamer.lof || true
	@$ rm -rf Beamer.snm || true
	@$ rm -rf Beamer.nav || true
	@$ rm -rf reports || true
	@$(ECHO) "Reporting Done ..."
