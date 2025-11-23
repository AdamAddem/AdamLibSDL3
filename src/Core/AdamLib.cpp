#include "WindowInternal.hpp"
#include <AdamLib/Core/AdamLib.hpp>
#include <AdamLib/Core/Window.hpp>
#include <AdamLib/Nodes/Node.hpp>
#include <SDL3/SDL.h>
#include <SDL3/SDL_timer.h>


uint64_t last_tick = 0;
uint64_t current_tick = 0;

double AdamLib::update()
{
  last_tick = current_tick;
  current_tick = SDL_GetTicks();
  return (current_tick - last_tick) / 1000.0;
}


void AdamLib::initialize()
{
  static bool initialized = false;
  if(initialized) return;

  SDL_Init(SDL_INIT_VIDEO);
  GameWindow::createWindow("AdamLib is dead! Long Live AdamLibSDL3 Port!");
  initialized = true;
}

void AdamLib::quit()
{
  SDL_Quit();
}