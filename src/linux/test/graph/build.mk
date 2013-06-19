obj-linux-test_graph += foo/graph.o
obj-linux-test_graph += linux/test/graph/test_graph.o
obj-linux-test_graph += kernel/math/fx_math.o
lib-linux-test_graph += -lm
bin-linux += test_graph
