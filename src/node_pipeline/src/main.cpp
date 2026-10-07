#include <exception>
#include <iostream>
namespace node_pipeline { namespace detail { int runGenerated(int argc, char** argv); }}
int main(int argc, char** argv) {
  try {return node_pipeline::detail::runGenerated(argc,argv);}
  catch(const std::exception& error) {std::cerr << "pipeline: " << error.what() << '\n';return 1;}
}
