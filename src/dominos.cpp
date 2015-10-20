/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 251 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * 
 */


#include "cs251_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
	#include <GLUT/glut.h>
#else
	#include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs251
{
  /**  The is the constructor 
   * This is the documentation block for the constructor.
   */ 
  
 

/*!  block object
       * \brief This is the class to create rectangular blocks
       */
class block{
public:
  b2Body* body;
  b2BodyDef bd;

  block(float h,float w,float x,float y,int a, b2World* m_world,int s_d=1,float  den=1.0f,float friction=0.0f){

    b2PolygonShape shape;
      shape.SetAsBox(h, w);
  
      
      bd.angle = a; //set the starting angle
      bd.position.Set(x, y);
      if(s_d==1)bd.type = b2_dynamicBody;
      if(s_d==0)bd.type = b2_staticBody;
      body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = den;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

  }
};




/*!  sphere object
       * \brief This is the class to create spheres
       */
class sphere
{


public:

    sphere(float radius,float density,float friction, float restitution, float x, float y,bool gravity, b2World* m_world,int s_d=1)
    {
        b2Body* sbody;
        b2CircleShape circle;
        circle.m_radius = radius;

        b2FixtureDef ballfd;
        ballfd.shape = &circle;
        ballfd.density = density;
        ballfd.friction = friction;
        ballfd.restitution = restitution;
        b2BodyDef ballbd;
        ballbd.type = b2_dynamicBody;
        if(s_d==0)ballbd.type = b2_staticBody; 
        ballbd.position.Set(x,y);
        sbody = m_world->CreateBody(&ballbd);
        sbody->CreateFixture(&ballfd);
        if(!gravity)
        {
            sbody->SetGravityScale(-1);
        }
    }
};




/*!  weld joint
       * \brief This is the class to create a fixed joint between 2 bodies
       */
class weld{
public:
  weld(b2Body* b1,b2Body* b2,float x,float y,b2World* m_world){
b2WeldJointDef weldJointDef;
weldJointDef.collideConnected = true;
b2Vec2 anchor;
      anchor.Set(x, y);
weldJointDef.Initialize(b1, b2, anchor);
m_world->CreateJoint(&weldJointDef);
}
};


/*!  revolute joint 
       * \brief This is the class to create a joint between 2 bodies which allows free rotation about the joint
       */

class rev_j{
public:
  rev_j(b2Body* b1,b2Body* b2,float x,float y,b2World* m_world,int mot=0){


b2RevoluteJointDef jd;


if(mot!=0){
jd.enableMotor = true;
jd.motorSpeed = mot;
jd.maxMotorTorque = 1000;
}
      b2Vec2 anchor;
      anchor.Set(x, y);
      jd.Initialize(b1, b2, anchor);
      m_world->CreateJoint(&jd);
 }
};



/*!  pulley joint
       * \brief This is the class to create a pulley with given 2 bodies
       */

class pulley_j{
// The pulley joint
     public:
pulley_j(b2Body* box1,b2Body* box2,float b1x,float b1y,float b2x,float b2y,float g1x,float g1y,float g2x,float g2y,b2World* m_world){
 
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(b1x, b1y); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(b2x, b2y); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(g1x, g1y); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(g2x, g2y); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);

}

};



/*!  open box object
       * \brief This is the class to create an open box
       */
class open_box{ 
public:
  b2Body* box1;
open_box(float x,float y, float l_density, float r_density, b2World *m_world)
    { 
block* b1=new block(2.0f,0.2f,0.0f+x,-1.9f+y,0,m_world,1,1.0f,0.5f);
block* b2=new block(0.2f,2.0f,2.0f+x,y,0, m_world,1,1.0f,0.5f);
block* b3=new block(0.2f,2.0f,-2.0f+x,y,0,m_world,1,1.0f,0.5f);
b1->body->SetFixedRotation(true);
weld* w=new weld(b1->body,b2->body,x+2.0f, y-1.9f,m_world);
w=new weld(b3->body,b1->body,x-2.0f, y-1.9f,m_world);
box1=b1->body;
    }
};



 dominos_t::dominos_t()
  {


block* b;
weld* w;
rev_j* r;
sphere* s;
    //Ground
    /*! \var b1 
     * \brief pointer to the body ground 
     */ 
    b2Body* b1;  
    {
      
      b2EdgeShape shape; 
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd); 
      b1->CreateFixture(&shape, 0.0f);
    }
          
    /*//Top horizontal shelf
    {
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);
	
      b2BodyDef bd;
      bd.position.Set(-31.0f, 30.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }
*/
   
   /*!  dominoes
       * \brief This is the set of 10 dominoes
       */

    //Dominos
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);
	
      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;
		
      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef bd;
	  bd.type = b2_dynamicBody;
	  bd.position.Set(-35.5f + 1.0f * i, 31.25f);
	  b2Body* body = m_world->CreateBody(&bd);
	  body->CreateFixture(&fd);
	}
    }
      

    //Another horizontal shelf
    {
      b2PolygonShape shape;
      shape.SetAsBox(7.0f, 0.25f, b2Vec2(-20.f,20.f), 0.0f);
	
      b2BodyDef bd;
      bd.position.Set(13.0f, -12.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }


//The heavy sphere on the platform
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 2.0;
  
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 0.005f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(-14.0f, 10.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }

    //The pendulum that knocks the dominos off
    {
      b2Body* b2;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 1.5f);
	  
	b2BodyDef bd;
	bd.position.Set(-36.5f, 28.0f);
	b2 = m_world->CreateBody(&bd);
	b2->CreateFixture(&shape, 10.0f);
      }
	
      b2Body* b4;
      {
	b2PolygonShape shape;
	shape.SetAsBox(0.25f, 0.25f);
	  
	b2BodyDef bd;
	bd.type = b2_dynamicBody;
	bd.position.Set(-40.0f, 33.0f);
	b4 = m_world->CreateBody(&bd);
	b4->CreateFixture(&shape, 2.0f);
      }
	
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-37.0f, 40.0f);
      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);
    }
      
    /*//The train of small spheres
    {
      b2Body* spherebody;
	
      b2CircleShape circle;
      circle.m_radius = 0.5;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
	
      for (int i = 0; i < 10; ++i)
	{
	  b2BodyDef ballbd;
	  ballbd.type = b2_dynamicBody;
	  ballbd.position.Set(-22.2f + i*1.0, 26.6f);
	  spherebody = m_world->CreateBody(&ballbd);
	  spherebody->CreateFixture(&ballfd);
	}
    }*/

//The heavy sphere on the platform
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 2.0;
  
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 150.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(-22.2f, 26.6f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }



    //The pulley system
    {
      float x=-10.0f;
      float y=25.0f;
     float x1=30.0f;
      float y1=5.0f;
      float x2=34.0f;
      float y2=6.0f;
     
     
open_box* ob=new open_box(x,y,10,10,m_world);
    
b=new block(4.0f,0.2f,x+x1,y+y1,0,m_world,0);
b2Body* b1=b->body;

b=new block(5.0f,0.2f,x+x2,y+y2,0,m_world,1,10);
b2Body* b2=b->body;
      // The pulley joint      
pulley_j* pj=new pulley_j(ob->box1,b2,x,y,x+x2,y+y2-1,x, y+5.0f,x+10.0f, y+5.0f,m_world);


//test
//b=new block(5.0f,0.2f,x,y+5.0f,0,m_world,1,10);

}


    //The revolving horizontal platform
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
	
      b2BodyDef bd;
      bd.position.Set(14.0f, 14.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(14.0f, 16.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }

    //The heavy sphere on the platform
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
	
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 50.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(14.0f, 18.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }



    //The see-saw system at the bottom
    {
      //The triangle wedge
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(-1,0);
      vertices[1].Set(1,0);
      vertices[2].Set(0,4.5);
      poly.Set(vertices, 3);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(-30.0f, 0.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);

      //The plank on top of the wedge
      b2PolygonShape shape;
      shape.SetAsBox(10.0f, 0.2f);
      b2BodyDef bd2;
      bd2.position.Set(-30.0f, 4.5f);
      bd2.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd2);
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape;
      body->CreateFixture(fd2);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-30.0f, 4.5f);
      jd.Initialize(sbody, body, anchor);
      m_world->CreateJoint(&jd);

      //The light box on the right side of the see-saw
      b2PolygonShape shape2;
      shape2.SetAsBox(1.0f, 1.0f);
      b2BodyDef bd3;
      bd3.position.Set(-38.0f, 5.0f);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.06f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);

     
    {

      //The revolving Launcher

      float x=-24.0f;
      float y=14.0f;
      float l=4.0f;
b=new block(l,0.2f,x,y,0,m_world,1);
b2Body* b1=b->body;
b=new block(0.01f,0.01f,x,y,0,m_world,0);
b2Body* b2=b->body;
r=new rev_j(b2,b1,x, y,m_world,0);
    
   b=new block(2.0f,0.2f,x+l+3.2f,y-1.6f,45,m_world,1);
  b2Body* b3=b->body;
  b=new block(2.0f,0.2f,x+l+1.2f,y-1.6f,-45,m_world,1);
b2Body* b4=b->body;
w=new weld(b1,b4,x+l, y,m_world);
w=new weld(b3,b4,x+l+2.0f, y-3.0f,m_world);


 b=new block(2.0f,0.2f,x-l-3.2f,y-1.6f,-45,m_world,1);
  b2Body* b5=b->body;
  b=new block(2.0f,0.2f,x-l-1.2f,y-1.6f,45,m_world,1);
b2Body* b6=b->body;
w=new weld(b1,b6,x-l, y,m_world);
w=new weld(b5,b6,x-l-2.0f, y-3.0f,m_world);
    }

{
  //Bucket of water
float x=30.0f;
float y=20.1f;
b=new block(3.0f,0.2f,x,y+2.2f,30,m_world,1);
b2Body* b1=b->body;
// base of the bucket b2
b=new block(1.5f,0.2f,x+2.0f,y,0,m_world,1);
b2Body* b2=b->body;
w=new weld(b1,b2,x+0.5f, y,m_world);
b=new block(3.0f,0.2f,x+4.0f,y+2.2f,-30,m_world,1);
b2Body* b4=b->body;
w=new weld(b4,b2,x+3.5f, y,m_world);
//lid of bucket b5
b=new block(2.0f,0.2f,x+3.2f,y+4.8f,0,m_world,1);
b2Body* b5=b->body;
w=new weld(b5,b4,x+4.6f, y+4.8f,m_world);

//water
for(int i=0;i<20;i++){
for(int j=0;j<15;j++){
b=new block(0.1f,0.1f,x+0.5f+0.2f*j,y+1.2f+0.2f*i,30,m_world,1);
}
}

//pivoting the bucket
b=new block(5.0f,0.2f,x+7.5f,y,0,m_world,0);
b2Body* b3=b->body;
r=new rev_j(b2,b3,x+2.5f, y,m_world);

}



{
//the pulley motor
  float x=22.0f;
float y=20.0f;
b=new block(3.0f,0.1f,x,y,0,m_world,1);
b2Body* b1=b->body;
b=new block(3.0f,0.1f,x,y,45,m_world,1);
b2Body* b2=b->body;
b=new block(3.0f,0.1f,x,y,90,m_world,1);
b2Body* b3=b->body;
b=new block(0.001f,0.001f,x,y,0,m_world,0);
b2Body* b4=b->body;


w=new weld(b1,b2,x, y,m_world);
w=new weld(b2,b3,x, y,m_world);
r=new rev_j(b4,b3,x, y,m_world,-1);
r=new rev_j(b4,b2,x, y,m_world,-1);
r=new rev_j(b4,b1,x, y,m_world,-1);

//test
//b=new block(0.5f,0.5f,x+2.2f,y+5.0f,0,m_world,1);
//b2Body* b7=b->body;

//w=new weld(b6,b5,x, y-10.0f,m_world);


}



{

//the Clock
float x=-15.0f;
float y=40.0f;
float radius=0.2f;
//the handle
b=new block(2.0f,0.1f,x,y,0,m_world,1);
b2Body* b1=b->body;
//the center
b=new block(0.001f,0.001f,x-2.0f,y,0,m_world,0);
b2Body* b2=b->body;
r=new rev_j(b1,b2,x-2.0f, y,m_world,1);
//the calibration
s=new sphere(radius,0.0f,0.0f,0.0f,x-2.0f,y+4.4f,0,m_world,0);
s=new sphere(radius,0.0f,0.0f,0.0f,x-2.0f,y-4.4f,0,m_world,0);
s=new sphere(radius,0.0f,0.0f,0.0f,x+2.2f,y,0,m_world,0);
s=new sphere(radius,0.0f,0.0f,0.0f,x-6.2f,y,0,m_world,0);
s=new sphere(radius,0.0f,0.0f,0.0f,x+1.0f,y+3.0f,0,m_world,0);
s=new sphere(radius,0.0f,0.0f,0.0f,x-5.0f,y+3.0f,0,m_world,0);
s=new sphere(radius,0.0f,0.0f,0.0f,x+1.0f,y-3.0f,0,m_world,0);
s=new sphere(radius,0.0f,0.0f,0.0f,x-5.0f,y-3.0f,0,m_world,0);

}


    }
  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
