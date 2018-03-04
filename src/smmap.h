#ifndef SMMAP_H
#define SMMAP_H
#include "map.h"
#include "harray2d.h"
#include "point.h"
#define SIGHT_INC 1

namespace LMapping {

struct PointAccumulator{
	typedef point<float> FloatPoint;
	/* before 
	PointAccumulator(int i=-1): acc(0,0), n(0), visits(0){assert(i==-1);}
	*/
	/*after begin*/
    //
	PointAccumulator(): acc(0,0), n(0), visits(0){}
	PointAccumulator(int i): acc(0,0), n(0), visits(0){assert(i==-1);}
	/*after end*/
    //
    inline void update(bool value, const Point& p=Point(0,0));  //更新某一点的被占据和扫描次数
    inline Point mean() const {return 1./n*Point(acc.x, acc.y);}    //返回当前点的当前点的值为=1/被占据次数
    inline operator double() const { return visits?(double)n*SIGHT_INC/(double)visits:-1; } //返回被占据的次数/扫描的次数
    inline void add(const PointAccumulator& p) {acc=acc+p.acc; n+=p.n; visits+=p.visits; }  //点相加
	static const PointAccumulator& Unknown();
	static PointAccumulator* unknown_ptr;
    //点
	FloatPoint acc;
    //每个点是否,
    //n为共有几次是被赋为占据的
    //vistis是共有几次扫描过得，包括有无占据
	int n, visits;
    //返回该点的信息熵
	inline double entropy() const;
};

void PointAccumulator::update(bool value, const Point& p)
{
    //如果value时真，则
    if (value)
    {
		acc.x+= static_cast<float>(p.x);
		acc.y+= static_cast<float>(p.y); 
		n++; 
		visits+=SIGHT_INC;
	} else
		visits++;
}

double PointAccumulator::entropy() const
{
    //如果未扫描次数，则返回-log(.5)
	if (!visits)
		return -log(.5);
    //如果扫描次数=被占据次数或被占据次数=0
	if (n==visits || n==0)
		return 0;
    //x为被占据次数/扫描次数
	double x=(double)n*SIGHT_INC/(double)visits;
    //信息熵为-[x*log(x)+(1-x)log(1-x)]
	return -( x*log(x)+ (1-x)*log(1-x) );
}

typedef Map<PointAccumulator,HierarchicalArray2D<PointAccumulator> > ScanMap;

};

#endif 
