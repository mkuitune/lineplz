#include <lnpz/lnpz.h>
#include<iostream>

void simpleSample() {
#if 0
	lnpz::Context c;
	c.addLine(0, 0, 1, 1);
#endif
	using namespace lnpz_linalg;
	using namespace std;
	double2 p(0, 1);
	double2 p2(0, 1);
	auto p3 = p + p2;
	double2x2 mat{ {0.7071067, -0.7071067}, {0.7071067, 0.7071067} };

	auto p4 = mul(mat, p3);

	cout << p4 << endl;

	double det = determinant(mat);
	cout << "Determinant " << det << endl;

	double4 quat1(1, 1, 1, 1);

	auto expres = qexp(quat1);
}


int main(int arc, char* argv[]) {
	simpleSample();
	return 0;
}