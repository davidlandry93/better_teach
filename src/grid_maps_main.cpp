
#include <iostream>
#include <string>
#include <vector>

#include "boost/program_options.hpp"

int main(int argc, char** argv)
{
  namespace po = boost::program_options;
  po::options_description desc("Options");

  desc.add_options()
    ("anchorpoint,a", po::value< std::string >(),
     "The anchor point to read")
    ("reading,r", po::value< std::string >(),
     "The reading to build the attraction bassin with")
    ("help,h", "produce help message");

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if(vm.count("help")) {
    std::cout << desc << std::endl;
    return 1;
  }

  if(vm.count("anchorpoint") && vm.count("reading")) {
    std::cout << "Ap: "
	      << vm["anchorpoint"].as< std::string >()
	      << "Re: "
	      << vm["reading"].as<  std::string >()
	      << std::endl;
  }
  
  std::cout << "Hello World!" << std::endl;

  return 0;
}
