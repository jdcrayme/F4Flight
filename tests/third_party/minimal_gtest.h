// Minimal gtest stub for dev. See previous file for full content.
#pragma once
#include <cmath>
#include <cstdio>
#include <vector>
#include <iostream>
namespace minimal_gtest {
struct TestCase { const char* name; void (*fn)(); };
inline std::vector<TestCase>& registry() { static std::vector<TestCase> r; return r; }
inline int& failure_count() { static int c=0; return c; }
inline int& test_count() { static int c=0; return c; }
struct Registrar { Registrar(const char* n, void(*f)()) { registry().push_back({n,f}); } };
}
#define TEST(s,n) static void s##_##n##_impl(); static ::minimal_gtest::Registrar s##_##n##_reg(#s "." #n, &s##_##n##_impl); static void s##_##n##_impl()
#define EXPECT_NEAR(a,b,e) do { double _a=(a),_b=(b),_e=(e),_d=std::fabs(_a-_b); ::minimal_gtest::test_count()++; if(_d>_e){::minimal_gtest::failure_count()++; std::printf("  FAIL %s:%d: EXPECT_NEAR(%g,%g,%g) diff %g\n",__FILE__,__LINE__,_a,_b,_e,_d);} } while(0)
#define EXPECT_DOUBLE_EQ(a,b) EXPECT_NEAR(a,b,1e-12)
#define EXPECT_EQ(a,b) do { ::minimal_gtest::test_count()++; if(!((a)==(b))){::minimal_gtest::failure_count()++; std::printf("  FAIL %s:%d: EXPECT_EQ\n",__FILE__,__LINE__);} } while(0)
#define EXPECT_NE(a,b) do { ::minimal_gtest::test_count()++; if((a)==(b)){::minimal_gtest::failure_count()++; std::printf("  FAIL %s:%d: EXPECT_NE\n",__FILE__,__LINE__);} } while(0)
#define EXPECT_LT(a,b) do { auto _a=(a),_b=(b); ::minimal_gtest::test_count()++; if(!(_a<_b)){::minimal_gtest::failure_count()++; std::printf("  FAIL %s:%d: EXPECT_LT(%g,%g)\n",__FILE__,__LINE__,(double)_a,(double)_b);} } while(0)
#define EXPECT_GT(a,b) do { auto _a=(a),_b=(b); ::minimal_gtest::test_count()++; if(!(_a>_b)){::minimal_gtest::failure_count()++; std::printf("  FAIL %s:%d: EXPECT_GT(%g,%g)\n",__FILE__,__LINE__,(double)_a,(double)_b);} } while(0)
#define EXPECT_TRUE(a) EXPECT_EQ(static_cast<bool>(a),true)
#define EXPECT_FALSE(a) EXPECT_EQ(static_cast<bool>(a),false)
#define EXPECT_THROW(a,exc) do { bool _c=false; try{a;}catch(const exc&){_c=true;} ::minimal_gtest::test_count()++; if(!_c){::minimal_gtest::failure_count()++; std::printf("  FAIL %s:%d: EXPECT_THROW\n",__FILE__,__LINE__);} } while(0)

// ASSERT_* macros -- same as EXPECT_* but return from the test function on failure
#define ASSERT_TRUE(a) do { if(!static_cast<bool>(a)){::minimal_gtest::failure_count()++; std::printf("  FAIL %s:%d: ASSERT_TRUE\n",__FILE__,__LINE__); return;} } while(0)
#define ASSERT_FALSE(a) do { if(static_cast<bool>(a)){::minimal_gtest::failure_count()++; std::printf("  FAIL %s:%d: ASSERT_FALSE\n",__FILE__,__LINE__); return;} } while(0)
#define ASSERT_EQ(a,b) do { if(!((a)==(b))){::minimal_gtest::failure_count()++; std::printf("  FAIL %s:%d: ASSERT_EQ\n",__FILE__,__LINE__); return;} } while(0)
#define ASSERT_NEAR(a,b,e) do { double _a=(a),_b=(b),_e=(e),_d=std::fabs(_a-_b); if(_d>_e){::minimal_gtest::failure_count()++; std::printf("  FAIL %s:%d: ASSERT_NEAR(%g,%g,%g) diff %g\n",__FILE__,__LINE__,_a,_b,_e,_d); return;} } while(0)
namespace testing { class Test { public: virtual ~Test()=default; virtual void SetUp(){} virtual void TearDown(){} virtual void TestBody()=0; void Run(){SetUp();TestBody();TearDown();} }; inline void InitGoogleTest(int*,char**){} }
#define TEST_F(f,n) class f##_##n##_Test : public f { public: void TestBody() override; }; static void f##_##n##_Test_register() { f##_##n##_Test t; t.Run(); } static ::minimal_gtest::Registrar f##_##n##_reg(#f "." #n, &f##_##n##_Test_register); void f##_##n##_Test::TestBody()
inline int RUN_ALL_TESTS() { int p=0,f=0; for(auto const& tc: ::minimal_gtest::registry()) { int f0=::minimal_gtest::failure_count(); std::printf("[ RUN      ] %s\n",tc.name); tc.fn(); int f1=::minimal_gtest::failure_count(); if(f1==f0){std::printf("[       OK ] %s\n",tc.name);p++;}else{std::printf("[  FAILED  ] %s\n",tc.name);f++;} } std::printf("\n%d tests passed, %d tests failed\n",p,f); return f==0?0:1; }
