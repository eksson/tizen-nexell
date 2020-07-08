# Note: plugindir is set in configure

plugin_LTLIBRARIES = libnxgstmeta.la

##############################################################################
# TODO: for the next set of variables, name the prefix if you named the .la, #
#  e.g. libmysomething.la => libmysomething_la_SOURCES                       #
#                            libmysomething_la_CFLAGS                        #
#                            libmysomething_la_LIBADD                        #
#                            libmysomething_la_LDFLAGS                       #
##############################################################################

# sources used to compile this plug-in
libnxgstmeta_la_SOURCES = \
	gstmmvideobuffermeta.c

# compiler and linker flags used to compile this plugin, set in configure.ac
libnxgstmeta_la_CFLAGS = \
	$(GST_CFLAGS)		\
	-I${includedir}

libnxgstmeta_la_LIBADD = \
	$(GST_LIBS)

libnxgstmeta_la_LDFLAGS =	\
	$(GST_PLUGIN_LDFLAGS)

libnxgstmeta_la_LIBTOOLFLAGS = --tag=disable-static

# headers we need but don't want installed
noinst_HEADERS =

nxgstmetaincludedir = ${includedir}
nxgstmetainclude_HEADERS = gstmmvideobuffermeta.h
