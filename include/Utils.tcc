namespace utils {

  extern std::chrono::steady_clock::time_point begin;
  template<typename TimeT >
  void tock(std::string name){
    std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
    std::cout << "TIME: " << name << " = " << std::chrono::duration_cast<TimeT>(end - begin).count() <<std::endl;
  };

}
