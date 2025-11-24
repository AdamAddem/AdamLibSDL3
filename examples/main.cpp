#include <AdamLib/Core/AdamLib.hpp>
#include <AdamLib/Core/Rendering.hpp>
#include <AdamLib/Core/Input.hpp>
#include <AdamLib/Nodes/Node.hpp>
#include "test.hpp"

#include <iostream>
using namespace AdamLib;

int main(int argc, char** argv)
{
  initialize();
  loadgame();
  Node& root = Node::getRoot();

  while(Input::processEvents())
  {
    root.process(update());
    Node::freeQueued();
    Renderer::render_all();
    
  }


  root.immediatelyKillAllChildren();
  quit();
  return 0;
}