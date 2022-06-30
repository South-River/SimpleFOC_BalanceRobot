#ifndef FILTER_H
#define FILTER_H

namespace Filter
{
    float ConstrainFilter(const float& input, const float& low, const float& up);

    class MeanFilter
    {
    public:
        void input(const float& in);
        float output();
        void reset();

    private:
        float sum=0;
        int cnt=0;
    };

    // First order low pass filter
    class FOLPF
    {
    public:
        FOLPF(const float& param = .05f);
        void input(const float& in);
        float output();
        void reset(const float& param = .05f);

    private:
        float alpha = .05f;
        float value = -1.f;
    };

    class MovingAverageFilter
    {
    public:
        MovingAverageFilter(const unsigned int &_size);
        void input(const float& in);
        float output();
        void reset(const unsigned int &_size);

    private:
        unsigned int head = 0;
        unsigned int tail = 0;
        float* queue;
        unsigned int size;
        unsigned int len=0;
    };
}

#endif
