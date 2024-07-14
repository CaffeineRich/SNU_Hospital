#pragma once
#include<vector>
#include<fstream>
#include<iostream>

template <typename T>			// ������ Ÿ���� �Һи��Ͽ� ���� ������ Ÿ�Կ� �°� �� �� �ֵ��� ����� Template

class Logging
{
private:
	std::ofstream fout;			// ���� ������� ���� ��ü
	std::vector<T> Data;		// ������ �����ϴ� vector
public:
	void keep(T data);			// ������ ����� �Լ�
	void Export();				// fstream�� ���� ������ ���
	Logging();					// ������ �� �Ҹ���
	Logging(std::string s);
	~Logging();
};

template <typename T> Logging<T>::Logging()
{
	// Constructor
	fout.open("log.csv", std::ios_base::out | std::ios_base::trunc);

	// ���� �ڸ����� �ش��ϴ� ������ ���¸� �� ó���� �Է��Ͽ� �����͸� ���� ������ �� �ִ�. 
	// ofstream�� csv ������ ��� Excel���·� �����͸� ������ �� ������ matlab���ε� Ȱ���� �����ϴ�.
	// ", " ������ Cell�� ������ ���� �ݵ�� �˾ƾ��Ѵ�.(, ���� �����̳� �ٸ��͵� �����ϳ� ,�� ���� ��Ȯ�ϴ�.)
	fout << "cnt" << ", " << "k.angle_last_target" << "," << "k.angle_actual" << std::endl;
}

template <typename T> Logging<T>::Logging(std::string s) {
	// Constructor
	std::string filename = s + ".csv";
	fout.open(filename, std::ios_base::out | std::ios_base::trunc);

	// ���� �ڸ����� �ش��ϴ� ������ ���¸� �� ó���� �Է��Ͽ� �����͸� ���� ������ �� �ִ�. 
	// ofstream�� csv ������ ��� Excel���·� �����͸� ������ �� ������ matlab���ε� Ȱ���� �����ϴ�.
	// ", " ������ Cell�� ������ ���� �ݵ�� �˾ƾ��Ѵ�.(, ���� �����̳� �ٸ��͵� �����ϳ� ,�� ���� ��Ȯ�ϴ�.)
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
	// ���Ϳ� ����� ���Ϳ�Ҵ� Data.at(0)[0] �� ������� ���ٰ��� >> ����� ���¸� ������ ���Ŀ� �°� �ٽ� �������Ͽ� ���� 
	for (int i = 0; i < Data.size(); i++) {
		if (i % 2 == 0) {
			fout << i / 2 + 1 << "," << Data.at(i) << "," << Data.at(i + 1) << std::endl;
		}
	}
}