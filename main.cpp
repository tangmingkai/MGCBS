#include <fstream>
#include <iostream>
#include <unordered_set>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>
#include <string>
#include <yaml-cpp/yaml.h>
#include "base/location.hpp"
#include "base/ts_state.hpp"
#include "base/agent.hpp"
#include "env/environment.hpp"
#include "base/timer.hpp"
#include "solver.hpp"

int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  std::string alg_name;
  desc.add_options()("help", "produce help message")(
      "algorithm,a", po::value<std::string>(&alg_name)->required(),
      ("algorithm name"+alg_list).c_str())(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)")("output,o",
                           po::value<std::string>(&outputFile)->required(),
                           "output file (YAML)");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  YAML::Node config = YAML::LoadFile(inputFile);
  std::cout << "load success" << std::endl;
  std::unordered_set<Location> obstacles;
  std::vector<Agent> agents;

  const auto& dim = config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();
  for (const auto& node : config["map"]["obstacles"]) {
    obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
  }
  for (const auto& node : config["agents"]) {
    const auto& agent_name = node["name"];
    const auto& start = node["start"];
    auto start_state = TSState(0, start[0].as<int>(), start[1].as<int>());
    std::vector<Location> goals;
    for (const auto& node1 : node["goals"]) {
      auto goal = Location(node1[0].as<int>(), node1[1].as<int>());
      goals.emplace_back(goal);
    }
    agents.emplace_back(
        Agent(agent_name.as<std::string>(), start_state, goals));
  }

  const Environment env(dimx, dimy, obstacles);
  Solver solver(env, alg_name);
  if (!solver.init_succ()) {
    std::cout << "Cannot build the algorithm, alg_list = " + alg_list
              << std::endl;
    return 0;
  }

  std::vector<PlanResult> solutions;
  Timer timer;
  bool success = solver.Solve(agents, &solutions);
  timer.Stop();

  Conflict tmp_conflict;
  if (env.GetFirstConflict(solutions, &tmp_conflict)) {
    std::cout << "Find conflict in the solutions, check the code." << std::endl;
    success = false;
  }
  if (success) {
    std::cout << "Planning successful! " << std::endl;
    int cost = 0;
    int makespan = 0;
    int max_time = 0;
    //  std::cout << solutions.size() << std::endl;
    for (const auto& s : solutions) {
      cost += s.cost;
      makespan = std::max<int>(makespan, s.cost);
      max_time = std::max(s.states.back().time, max_time);
    }
    // std::cout << cost << " " << makespan << " " << max_time << std::endl;
    std::ofstream out(outputFile);
    out << "method: " << alg_name << std::endl;
    out << "statistics:" << std::endl;
    out << "  cost: " << cost << std::endl;
    out << "  makespan: " << makespan << std::endl;
    out << "  runtime: " << timer.ElapsedSeconds() << std::endl;
    out << "schedule:" << std::endl;
    for (size_t a = 0; a < solutions.size(); ++a) {
      out << "  " << solutions[a].agent_name << ":" << std::endl;
      for (const auto& state : solutions[a].states) {
        out << "    - x: " << state.x << std::endl
            << "      y: " << state.y << std::endl
            << "      t: " << state.time << std::endl;
      }
      auto& last_state = solutions[a].states.back();
      for (int i = last_state.time + 1; i <= max_time; i++) {
        out << "    - x: " << last_state.x << std::endl
            << "      y: " << last_state.y << std::endl
            << "      t: " << i << std::endl;
      }
    }
  } else {
    std::cout << "Planning NOT successful!" << std::endl;
  }

  return 0;
}
