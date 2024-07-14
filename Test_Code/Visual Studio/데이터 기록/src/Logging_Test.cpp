#include <Logging.h>

double ch1 = 1, ch2 = 2;

int main()
{
    Logging<double> log;
    int i = 0;
    while (1)
	{
		if (i % 2 == 0) log.keep(ch1);
		else log.keep(ch2);
		i++;
		if (i > 49) {
			log.Export();
			break;
		}
    }
    return 0;                                                                                                           // 프로그램 종료
}