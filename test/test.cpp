#include <gtest/gtest.h>

#include <lnpz/lnpz_linalgd.h>

// Demonstrate some basic assertions.
TEST(MathTest, Span2DOperations) {
	// Expect two strings not to be equal.
	EXPECT_STRNE("hello", "world");
	// Expect equality.
	EXPECT_EQ(7 * 6, 42);

	using namespace lnpz_linalg;

	typedef vec<double, 2> p2_t;

	auto clus = clusteringradius<double>::Create(0.00001);

	p2_t a(0, 0);
	p2_t b(1, 0);
	auto ab = segment2<double>::Create(a, b);

	auto resabab = ab.intersect(ab);
	EXPECT_EQ(resabab.count, 2);

	p2_t aa(0.4, 0);
	p2_t c(0.5, 0);
	auto aac = segment2<double>::Create(aa, c);
	
	auto resabac = ab.intersect(aac);
	EXPECT_EQ(resabac.count, 2);
	
	p2_t d(0.5,0.1);
	auto ad = segment2<double>::Create(a, d);
	auto resabad = ab.intersect(ad);
	EXPECT_EQ(resabad.count, 1);
	
	p2_t e(0.5, -0.5);
	p2_t f(0.5, 0.5);
	auto ef = segment2<double>::Create(e, f);

	auto resabef = ab.intersect(ef);
	EXPECT_EQ(resabef.count, 1);

}