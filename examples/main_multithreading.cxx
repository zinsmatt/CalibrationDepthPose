//=========================================================================
//
// Copyright 2019 Kitware, Inc.
// Author: Matthieu Zins
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//=========================================================================


#include <iostream>
#include <chrono>
#include <cmath>

#include <CalibrationDepthPose/threadPool.h>



int main()
{

  int waiting_duration = 5;
  int nb_tasks = 16 * 5 * 10;
  int nb_workers = 16 * 5 * 10;

  CalibrationDepthPose::ThreadPool pool(nb_workers);

  std::vector<std::future<int>> results;

//  std::vector<std::thread> threads;
//  std::vector<int> results2;
//  std::mutex mtx;


  std::chrono::system_clock::time_point start_t = std::chrono::system_clock::now();

  for (int i = 0; i < nb_tasks; ++i)
  {
    auto fct = [i, waiting_duration]() {
      std::cout << "Hello " << i << std::endl;
      std::this_thread::sleep_for(std::chrono::seconds(waiting_duration));
      std::cout << "World " << i << std::endl;
      return i * i;
    };
    results.emplace_back(
          pool.enqueue(fct)
      );

//    threads.emplace_back([i, waiting_duration, &results2, &mtx]() {
//      std::cout << "Hello " << i << std::endl;
//      std::chrono::system_clock::time_point start_t = std::chrono::system_clock::now();
//      int k = 0;
//      std::vector<int> res(50000);
//      while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - start_t).count() < 5)
//      {
//        res[k % 50000] = k % 5000;
//      }
//      std::cout << "World " << i << std::endl;
//      {
//        std::unique_lock<std::mutex> lock(mtx);
//        results2.push_back(i*i);
//      }
//    });
  }



  for (auto&& r : results)
  {
    std::cout << r.get() << " ";
  }

//  for (auto& th : threads)
//  {
//    th.join();
//    std::cout << 1 << " ";
//  }
  std::cout << std::endl;

  std::chrono::system_clock::time_point end_t = std::chrono::system_clock::now();

  double real_duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count();
  double expected_duration = 1000 * waiting_duration * nb_tasks;

  std::cout << "Duration = " << real_duration << " [ms]" << std::endl;
  std::cout << "Expected time = " << expected_duration << " [ms] " << std::endl;

  std::cout << "Ratio = " << expected_duration / real_duration << std::endl;
  return 0;
}
