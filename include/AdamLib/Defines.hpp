#pragma once


//Character to be appended to non-unique node name. (Ex: if adding child 'box' to node who already has child 'box', append this character then add)
#define CHAR_TO_APPEND '.'


//Collision Draw Color.
#define CDC_R 0
#define CDC_G 255
#define CDC_B 255
#define CDC_A 255


//CollisionTree AABB Thickness Modifier
#define SKIN_THICKNESS 0.05

//Maximum quantity of Signals allowed, stupid stupid stupid reason for this 
#define MAXIMUM_SIGNALS 1024

//Number of collision layers. Shouldn't need more than 8 normally.
#define NUM_COLLISION_LAYERS 8