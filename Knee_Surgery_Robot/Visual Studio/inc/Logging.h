#pragma once
#include<vector>
#include<fstream>
#include<iostream>

template <typename T>			// 데이터 타입이 불분명하여 너의 데이터 타입에 맞게 쓸 수 있도록 만드는 Template

class Logging
{
private:
	std::ofstream fout;			// 파일 입출력을 위한 객체
	std::vector<T> Data;		// 데이터 저장하는 vector
public:
	void keep(T data);			// 데이터 저장용 함수
	void Export();				// fstream을 통한 데이터 출력
	Logging();					// 생성자 및 소멸자
	Logging(std::string s);
	~Logging();
};

template <typename T> Logging<T>::Logging()
{
	// Constructor
	fout.open("log.csv", std::ios_base::out | std::ios_base::trunc);

	// 각각 자리마다 해당하는 데이터 형태를 맨 처음에 입력하여 데이터를 쉽게 정리할 수 있다. 
	// ofstream의 csv 파일의 경우 Excel형태로 데이터를 가공할 수 있으며 matlab으로도 활용이 가능하다.
	// ", " 단위로 Cell을 나누는 것을 반드시 알아야한다.(, 말고도 공백이나 다른것도 가능하나 ,가 가장 명확하다.)
	fout << "cnt" << ", " << "k.angle_last_target" << "," << "k.angle_actual" << std::endl;
}

template <typename T> Logging<T>::Logging(std::string s) {
	// Constructor
	std::string filename = s + ".csv";
	fout.open(filename, std::ios_base::out | std::ios_base::trunc);

	// 각각 자리마다 해당하는 데이터 형태를 맨 처음에 입력하여 데이터를 쉽게 정리할 수 있다. 
	// ofstream의 csv 파일의 경우 Excel형태로 데이터를 가공할 수 있으며 matlab으로도 활용이 가능하다.
	// ", " 단위로 Cell을 나누는 것을 반드시 알아야한다.(, 말고도 공백이나 다른것도 가능하나 ,가 가장 명확하다.)
	fout << "cnt" << ", " << "k.angle_last_target" << "," << "k.angle_actual" << std::endl;
}

template <typename T> Logging<T>::~Logging() {
	/// Destructor
}

template <typename T> void Logging<T>::keep(T data) {
	Data.push_back(data);
}

template <typename T> void Logging<T>::Export()
{
	// 벡터에 저장된 백터요소는 Data.at(0)[0] 의 방식으로 접근가능 >> 저장된 형태를 데이터 형식에 맞게 다시 재조정하여 저장 
	for (int i = 0; i < Data.size(); i++) {
		if (i % 2 == 0) {
			fout << i / 2 + 1 << "," << Data.at(i) << "," << Data.at(i + 1) << std::endl;
		}
	}
}