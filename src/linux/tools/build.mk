###
# Générateur de Sinus
###
obj-linux-sin_table_gen += linux/tools/sin_generator/sin_table_gen.o
lib-linux-sin_table_gen += -lm
bin-linux += sin_table_gen

###
# Générateur de graphe
###
obj-linux-graph_gen += linux/tools/graph_generator/graph_gen.o
lib-linux-graph_gen += -lm
bin-linux += graph_gen