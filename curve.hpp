//T�����߸ģ�����ֵΪ�������ٶȣ�����ٶȣ�Ŀ��λ�ã�
//tc_ָ�˶���Ŀ�����ʱ�䣻
//tm_ָ������ٶ��˶�����ʱ�䣻
//v_Ϊ����˶��ٶȣ�
//a_Ϊ����˶����ٶȣ�
//ta_ָ����ʱ�䣻
//p_ָ�����˶�λ�ã�
//pa_ָ�Ӽ�������Ҫ�˶���λ�á�
class TCurve2
{
private:
    double tc_;
    double tm_;
    double v_;
    double a_;
    double ta_;
    double p_;
    double pa_;

public:
    auto getTCurve(int count) -> double;
    auto getCurveParam() -> void;
    auto getTc() -> double { return tc_; };
    TCurve2(double a, double v, double p) { a_ = a; v_ = v; p_ = p; }
    ~TCurve2() {}
};