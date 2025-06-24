#include "display_backend/raylib_display_backend.h"
#include "game_logic.h"


int main() {
    RaylibDisplayBackend backend = RaylibDisplayBackend();
    GameLogic gameLogic(backend);
    return gameLogic.start();
}
