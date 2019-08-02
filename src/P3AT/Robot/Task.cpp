//***************************************************************************
// Copyright 2007-2019 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: renan                                                            *
//***************************************************************************

// Standard headers
#include <math.h>   // For M_PI definition

// DUNE headers.
#include <DUNE/DUNE.hpp>
#include <DUNE/Time.hpp>

// ARIA library headers
#include <aria/include/Aria.h>

namespace P3AT
{
  //! Task for running Pioneer 3 All-Terrain (P3AT) robot drivers
  //! for movement and localization (based on wheel encoders)
  //!
  //! @author Renan Maidana (rgmaidana)
  namespace Robot
  {
    using DUNE_NAMESPACES;

    struct Task: public DUNE::Tasks::Task
    {
      // IMC message for the robot's velocity (inbound)
      IMC::DesiredVelocity *robotVel = new IMC::DesiredVelocity();
      // IMC message for the robot's position (outbound)
      IMC::EstimatedState *robotState = new IMC::EstimatedState();

      // Robot and connection utilities are resources
      // to be acquired and initialized through DUNE
      ArRobot *robot;
      ArArgumentBuilder *args;
      ArArgumentParser *parser;
      ArRobotConnector *conn;

      // From DUNE/Time.hpp library, utilities for counting time duration
      // In this case, we use it to figure out when to reset the robot speeds,
      // so the robot will stop when it hasn't received any new commands in some time
      float stop_time;
      Time::Delta stop_delta;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx), 
        robot(NULL), args(NULL), parser(NULL), conn(NULL)
      {
        // "Subscriber" to velocity messages
        bind<IMC::DesiredVelocity>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
        robot = new ArRobot();
        args = new ArArgumentBuilder();
        parser = new ArArgumentParser(args);
        conn = new ArRobotConnector(parser, robot);
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
        Aria::init();

        parser->loadDefaultArguments();         // Load default values for robot
        args->add("-robotPort /dev/ttyUSB0");   // Set robot serial port to ttyUSB0
        args->add("-robotBaud 9600");           // Set robot default baudrate (9600 for most robots, 57600 for Pioneer LX)

        // Parse robot connection arguments
        if(!conn->parseArgs()){
          inf("Error: Unknown robot arguments.\n");
          Aria::exit(0);
          exit(1);
        }

        // Attempt connection with the robot
        if(!conn->connectRobot()){
          inf("Error: Could not connect to robot. Check serial ports and try again!\n");
          Aria::exit(0);
          exit(1);
        }

        // Run robot in asynchronous mode
        robot->runAsync(true);
        
        robot->lock();                        // Lock robot during motor setup
        robot->setAbsoluteMaxTransVel(500);   // Set maximum robot translational speed  (500 mm/s)
        robot->setAbsoluteMaxRotVel(45);      // Set maximum robot rotational speed (45 deg/s)
        robot->comInt(ArCommands::ENABLE, 1); // Enable motors
        robot->setVel(0);                     // Robot starts in a stopped state
        robot->setRotVel(0);                  
        robot->unlock();                      // Unlock robot

        // Clears time for stopping the robot
        stop_delta.clear();
        stop_time = 0.0;
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
        Memory::clear(conn);
        Memory::clear(parser);
        Memory::clear(args);
        Memory::clear(robot);
      }

      // Callback to receive teleop velocity messages
      void consume(const IMC::DesiredVelocity* msg){
        // inf("Linear velocity: %f\n", msg->u);
        // inf("Angular velocity: %f\n", msg->r);

        // Set velocities as new messages arrive
        robot->lock();
        robot->setVel(msg->u * 1000.0);          // Convert from m/s to mm/s
        robot->setRotVel(msg->r * 180.0/M_PI);   // Convert from rad/s to deg/s
        robot->unlock();

        // Clears time for stopping the robot
        stop_delta.clear();
        stop_time = 0.0;
      }

      //! Main loop.
      void
      onMain(void)
      {
        ArPose pose;             // Object to obtain the robot pose

        // Run while dune is active
        while (!stopping())
        {
          // Update and dispatch robot estimated state
          pose = robot->getPose();
          robotState->x = pose.getX() / 1000.0;
          robotState->y = pose.getY() / 1000.0;
          robotState->psi = pose.getTh() * M_PI/180.0;
          dispatch(robotState);

          // Robot velocity is set at the consume callback, when it receives a new DesiredVelocity message

          // Stop the robot if no new velocities have been received for 400 miliseconds
          float delta = stop_delta.getDelta();
          if (delta > 0.0)    // Check for a valid delta
            stop_time += delta;
          if(stop_time > 0.4f){
            robot->lock();
            robot->setVel(0);
            robot->setRotVel(0);
            robot->unlock();
          }

          waitForMessages(0.1);                         // Sleep for 100 ms (10 Hz)
        }

        // Stop the robot
        robot->setVel(0);
        robot->setRotVel(0);

        // Close Aria
        Aria::exit(0);
      }
    };
  }
}

DUNE_TASK