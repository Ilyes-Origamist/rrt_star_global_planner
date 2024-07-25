#include <armadillo>


Approach:

-Create a class "PathAgent" representing one agent, constructor takes the path, then transform the path to a Eign vector to manipulate it.

-Create a vector containing shared pointers, each pointing to an object of the class PathAgent



Class: PathAgent



Constructor(Path, id)

Methods:
cost:
updateAgent

Data Members:
path (=Path from input): list of pairs 
agent: vector of length 2*size(Path)




// parameters
max_iter; // max number of iterations = N
num_agents; // number of agents = Ng

// class
WOAAgent
// constructor:
WOAAgent(const std::list<std::pair<float, float>> initial_path, int id)
// methods

// modifies X using A, C and X_search
// X_sear either equal to X_best or X_rand
void circularUpdate(PathAgent& search_agent);{
  arma::vec X;
  D=abs(C*X_search-X);
  X=X_search-A*D;
}
float cost()
// returns the cost of the path


// data members

public:
  arma::vec X: the vector representing the path
  float A: updated in each iteration
  float C: updated in each iteration
  
private:
  D: computed from A, C and X_search using the method updateAgent(X_search)

