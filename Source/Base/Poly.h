#pragma once
namespace Framework
{
	namespace Base
	{

struct COMMON_API PolynomialEquation_forth_Props
{
	void Init()
	{
		memset(&Term,0,sizeof(PolynomialEquation_forth_Props));
		Term[1]=1.0;
	}
	double Term[5];
#if 0
	virtual void LoadFromScript(Scripting::Script& script,const char FieldName[])
	{
		const char* err=NULL;
		err = script.GetFieldTable(FieldName);
		if (!err)
		{
			err = script.GetField("c", NULL, NULL,&Term[0]);
			ASSERT_MSG(!err, err);
			err = script.GetField("t1", NULL, NULL,&Term[1]);
			ASSERT_MSG(!err, err);
			err = script.GetField("t2", NULL, NULL,&Term[2]);
			ASSERT_MSG(!err, err);
			err = script.GetField("t3", NULL, NULL,&Term[3]);
			ASSERT_MSG(!err, err);
			err = script.GetField("t4", NULL, NULL,&Term[4]);
			ASSERT_MSG(!err, err);
			script.Pop();
		}
	}
#endif
};

class COMMON_API PolynomialEquation_forth
{
	public:
		PolynomialEquation_forth()
		{
			memset(&m_PolyProps,0,sizeof(PolynomialEquation_forth_Props));
			m_PolyProps.Term[1]=1.0;
		}
		void Initialize(const PolynomialEquation_forth_Props *props=NULL)
		{
			if (props)
				m_PolyProps=*props;
		}

		/// \note these equations most-likely will not be symmetrical in the negative range so we'll work with the 
		///positive range and restore the sign
		__inline double operator() (double x)
		{
			double y=fabs(x);
			const double *c=m_PolyProps.Term;
			const double x2=y*y;
			const double x3=y*x2;
			const double x4=x2*x2;
			y = (c[4]*x4) + (c[3]*x3) + (c[2]*x2) + (c[1]*y) + c[0]; 
			const double result=(x<0)?-y:y;
			return result;
		}

		/// This version of the function is identical to previous but also includes auto clipping prior to restoring the sign
		/// \param clip_value used to auto clip before restoring sign this is typically set to 1.0
		__inline double operator() (double x,double clip_value)
		{
			double y=fabs(x);
			const double *c=m_PolyProps.Term;
			const double x2=y*y;
			const double x3=y*x2;
			const double x4=x2*x2;
			y = (c[4]*x4) + (c[3]*x3) + (c[2]*x2) + (c[1]*y) + c[0]; 
			y=(y<clip_value)?y:clip_value;  //min operation
			const double result=(x<0)?-y:y;
			return result;
		}

	private:
		PolynomialEquation_forth_Props m_PolyProps;
};
}}