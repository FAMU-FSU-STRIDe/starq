#include <stdio.h>

#include "starq/unitree/unitree_a1_mujoco_robot.hpp"
#include "starq/osqp/osqp.hpp"
#include "starq/mpc/mpc_controller.hpp"
#include "starq/planning/planning_solver.hpp"
#include "starq/starq/starq_planning_model.hpp"

using namespace starq;
using namespace starq::mpc;
using namespace starq::osqp;
using namespace starq::unitree;
using namespace starq::planning;

void printSolution(OSQP::Ptr osqp, MPCConfiguration::Ptr mpc_config);

int main(void)
{

    // Create Unitree A1 robot
    auto robot = std::make_shared<UnitreeA1MuJoCoRobot>();
    printf("UnitreeA1MuJoCoRobot created\n");

    // Load walk gait from file
    Gait::Ptr walk_gait = std::make_shared<Gait>();
    walk_gait->load("/home/nvidia/starq_ws/src/starq/gaits/walk.txt");
    printf("Walk Gait loaded\n");

    // Set gait to walk forward at constant velocity
    walk_gait->setVelocity(Vector3(0, 0, 0), Vector3(0, 0, 0));
    walk_gait->setFrequency(3.0);

    // Load stand gait from file
    Gait::Ptr stand_gait = std::make_shared<Gait>();
    stand_gait->load("/home/nvidia/starq_ws/src/starq/gaits/stand.txt");
    printf("Stand Gait loaded\n");

    // Stand still at default height
    stand_gait->setPose(Vector3(0, 0, UNITREE_A1_STAND_HEIGHT), Vector3(0, 0, 0));
    stand_gait->setFrequency(10.0);

    // Create MPC Configuration object based on the robot
    MPCConfiguration::Ptr mpc_config = std::make_shared<MPCConfiguration>(robot->getLegs(),
                                                                          robot->getRobotParameters(),
                                                                          robot->getLocalization());

    // Set MPC parameters
    mpc_config->setTimeStep(milliseconds(50));
    mpc_config->setWindowSize(21);
    printf("MPCConfiguration created\n");

    // Create OSQP solver object
    OSQP::Ptr osqp = std::make_shared<OSQP>();
    printf("OSQP created\n");

    // Set OSQP parameters
    osqp->getSettings()->verbose = false;
    osqp->getSettings()->max_iter = 2000;
    osqp->getSettings()->polishing = true;
    osqp->getSettings()->warm_starting = true;

    // Create MPC Controller object
    // MPC Controller is responsible for running the MPC solver and sending the leg commands to the robot
    MPCController::Ptr mpc_controller = std::make_shared<MPCController>(mpc_config, osqp,
                                                                        robot->getLegCommandPublisher());

    // Create starq planning model
    STARQPlanningModel::Ptr model = std::make_shared<STARQPlanningModel>(robot->getLocalization());
    model->setGoalState(Vector3(1.0, 1.0, 0.0));
    model->setGoalThreshold(0.1);
    printf("Model created.\n");

    // Create STARQ planning solver
    PlanSolver::Ptr solver = std::make_shared<PlanSolver>();
    printf("Solver created.\n");

    // Create plan configuration
    PlanConfiguration::Ptr config = std::make_shared<PlanConfiguration>();
    config->dx = Vector3(0.1, 0.1, M_PI / 16.0);
    config->dt = 0.25;
    config->time_limit = milliseconds(500);
    printf("Configuration created.\n");

    // Start simulation
    robot->startSimulation();
    printf("Simulation started\n");

    // Set foot positions to default locations
    for (uint8_t id = 0; id < UNITREE_A1_NUM_LEGS; id++)
    {
        robot->setFootPosition(id, robot->getRobotParameters()->getDefaultFootLocations()[id]);
    }
    printf("Holding foot position for 5 seconds...\n");

    // Wait for simulation to settle
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Start MPC Controller in a separate thread
    mpc_controller->start();
    printf("MPCController started\n");

    // Run standing gait for 5 seconds
    printf("Standing for 5 seconds...\n");
    mpc_config->setNextGait(stand_gait);
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Transition to walking gait
    printf("Walking...\n");
    mpc_config->setNextGait(walk_gait);

    printf("Starting Planner\n");
    while (robot->isSimulationOpen())
    {

        // Print position
        const Vector3 position = robot->getLocalization()->getCurrentPosition();
        const Vector3 orientation = robot->getLocalization()->getCurrentOrientation();
        printf("Current position: x: %f y: %f th: %f", position.x(), position.y(), orientation.z());

        // Solve
        PlanResults::Ptr results = solver->solve(config, model);

        // Send command
        if (results->node_path.size() > 0)
        {
            VectorX u = results->node_path[1]->u;
            printf("  ->  Sending command: vx: %f vy: %f w: %f\n", u(0), u(1), u(2));
            walk_gait->setVelocity(Vector3(u(0), u(1), 0), Vector3(0, 0, u(2)));
        }
        else
        {
            printf("  ->  No solution found\n");
            walk_gait->setVelocity(Vector3(0, 0, 0), Vector3(0, 0, 0));
        }

        // Sleep for 10 milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Wait for simulation to close
    robot->waitForSimulation();
    printf("Simulation closed\n");

    // Cleanup
    robot->cleanup();

    printf("Done\n");
    return 0;
}