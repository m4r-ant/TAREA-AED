#include "gui.h"

int main() {
    GUI gui("nodes_small.csv", "edges_small.csv");
    gui.main_loop();
    return 0;
}
