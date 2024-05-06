#include <stdio.h>

#include "starq/system_localization.hpp"
#include "starq/planning/planning_solver.hpp"
#include "starq/starq/starq_planning_model.hpp"

using namespace starq;
using namespace starq::planning;

int main()
{
    // Create localization
    SystemLocalization::Ptr localization = std::make_shared<SystemLocalization>();

    // Create starq planning model
    STARQPlanningModel::Ptr model = std::make_shared<STARQPlanningModel>(localization);
    model->setGoalState(Vector3(0.0, -2.0, -M_PI / 2.0));
    model->setGoalThreshold(0.10);
    printf("Model created.\n");

    // Create STARQ planning solver
    PlanSolver::Ptr solver = std::make_shared<PlanSolver>();
    printf("Solver created.\n");

    // Create plan configuration
    PlanConfiguration::Ptr config = std::make_shared<PlanConfiguration>();
    config->dx = Vector3(0.05, 0.05, M_PI / 64.0);
    config->dt = 0.15;
    config->time_limit = milliseconds(2000);
    config->max_iterations = 100000;
    config->max_generations = 200;
    printf("Configuration created.\n");

    // Solve
    PlanResults::Ptr results = solver->solve(config, model);
    printf("Solved.\n");

    // Print results
    printf("Exit code: %d\n", results->exit_code);
    printf("Time elapsed: %lu ms\n", results->time_elapsed.count());
    printf("Iterations: %d\n", results->iterations);
    printf("Cost: %f\n", results->cost);
    for (auto node : results->node_path)
    {
        printf("Node: x: %f y: %f th: %f g: %f h: %f f: %f\n", node->x(0), node->x(1), node->x(2), node->g, node->h, node->f);
    }

    // Save nodes
    solver->saveNodes("/home/nvidia/starq_ws/src/logging/nodes.txt");

    printf("Done.\n");
    return 0;
}