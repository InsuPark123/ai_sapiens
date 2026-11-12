#include <gtest/gtest.h>
#include <cstdio>
#include <fstream>
#include <unistd.h>
#include "ai_sapiens_sim2real/policy/motion_reference.hpp"
using ai_sapiens_sim2real::MotionReference;

TEST(SuperMimicReference, WindowsClampAtEndAndRejectMissingOrMalformedData)
{
  char path[] = "/tmp/supermimic_reference_XXXXXX";
  int fd = mkstemp(path); ASSERT_GE(fd, 0); close(fd);
  const std::string sidecar = std::string(path) + ".supermimic";
  struct Cleanup {std::string a, b;
    ~Cleanup()
    {
      std::remove(a.c_str());std::remove(b.c_str());
    }
     } cleanup{path, sidecar};
  std::ofstream(path) << "0,0,0,0,0,0,1,0\n0,0,0,0,0,0,1,0\n";
  MotionReference missing(path, 50, {"joint"});
  EXPECT_THROW(missing.supermimic_goal(), std::runtime_error);
  {std::ofstream out(sidecar); for(int f = 0; f < 2; ++f) {
      for(int i = 0; i < 56; ++i) {
        if(i) {
          out << ',';
        }
        out << (f * 100 + i);
      }
      out << '\n';
    }
     }
  MotionReference ref(path, 50, {"joint"});
  auto goal = ref.supermimic_goal(); ASSERT_EQ(goal.size(), 560U);
  EXPECT_FLOAT_EQ(goal[0], 0); EXPECT_FLOAT_EQ(goal[55], 55); EXPECT_FLOAT_EQ(goal[56], 100);
    EXPECT_FLOAT_EQ(goal[559], 155);
  ref.seek(.02); goal = ref.supermimic_goal(); EXPECT_FLOAT_EQ(goal[0], 100);
    EXPECT_FLOAT_EQ(goal[559], 155);
  std::ofstream(sidecar) << "invalid\n";
  EXPECT_THROW(MotionReference(path, 50, {"joint"}), std::runtime_error);
}
