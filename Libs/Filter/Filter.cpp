#include "Filter.h"

namespace Filter
{
    float ConstrainFilter(const float& input, const float& low, const float& up)
    {
        float output = (input>low)?((input<up)?input:up):low;
        return output;
    }

    void MeanFilter::input(const float& in)
    {
        sum += in;
        cnt += 1;
    }

    float MeanFilter::output()
    {
        if(cnt <= 0)
            return -1;
        return sum/cnt;
    }

    void MeanFilter::reset()
    {
        sum=0;
        cnt=0;
    }

    FOLPF::FOLPF(const float& param)
    {
        alpha = param;  
    }

    void FOLPF::input(const float& in)
    {
        if(value == -1.f)
        {
            value = in;
        }
        else
        {
            value = alpha*in + (1.f - alpha) * value;
        }
        return;
    }

    float FOLPF::output()
    {
        return value;
    }

    void FOLPF::reset(const float& param)
    {
        alpha=param;
        value=-1.f;
    }

    MovingAverageFilter::MovingAverageFilter(const unsigned int &_size)
    {
        if(_size<=0)
        {
            return;
        }
        else if(_size>0)
        {
            float _queue[_size];
            queue=_queue;
            head=0;
            tail=0;
            size=_size;

            for(int i=0; i<_size; i++)
            {
                _queue[i]=0.f;
            }
        }
    }

    void MovingAverageFilter::input(const float& in)
    {
        if((tail==head)&&(len!=0))
        {
            head=(head+1)%size;
            queue[tail]=in;
            tail=head;
        }
        else if((tail+1)%size!=head)
        {
            queue[tail++]=in;
            len++;
        }
    }

    float MovingAverageFilter::output()
    {
        if(len<=0)
        {
            return 0.f;
        }
        else
        {
            float value=0.f;

            for(int i=0; i<len; i++)
            {
                value += queue[head%size];
            }
            value/=size;

            return value;
        }
    }

    void MovingAverageFilter::reset(const unsigned int &_size)
    {
        if(_size<=0)
        {
            return;
        }
        else if(_size>0)
        {
            float _queue[_size];
            queue=_queue;
            head=0;
            tail=0;
            size=_size;

            for(int i=0; i<_size; i++)
            {
                _queue[i]=0.f;
            }
        }
    }
}