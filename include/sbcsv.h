#pragma once

#include <base/typedefs.h>

#include <sbtokenizer.h>

namespace Scenebuilder{;

/* CSV�ǂݍ��݃N���X
	- �t�@�C���R���e���c�𕶎���Ƃ��ăg�[�N�i�C�Y���C�l���擾���ɖ���^�ϊ�����D
	- �f���~�^���A�������ꍇ�C�P�̗�Ƃ��Ĉ���
	- ���s���A�������ꍇ�C�P�̍s�Ƃ��Ĉ����D
 */
class CsvReader{
public:
	struct Row{
		vector<string_iterator_pair>	col;
	};
	vector<Row>	row;

	string contents;			///< contents string

	Tokenizer rowIt;
	Tokenizer colIt;

public:
	/// �N���A
	void Clear(){
		contents.clear();
		row.clear();
	}

	/// �ǂݍ���
	bool Read(const string& filename, const string& delim){
		ifstream ifs;
		ifs.open(filename, ifstream::in | ifstream::binary);

		if(!ifs.is_open())
			return false;

		ifs.seekg(0, ifs.end);
		int len = (int)ifs.tellg();
		ifs.seekg(0, ifs.beg);

		contents.resize(len);
		ifs.read(&contents[0], len);
	
		ifs.close();
	
		// �܂��s���𐔂���
		rowIt.Set(contents, "\n", false);
		int nrow = 0;
		for(; !rowIt.IsEnd(); rowIt.Next())
			nrow++;
		row.resize(nrow);

		rowIt.Set(contents, "\n", false);
		for(int i = 0; !rowIt.IsEnd(); rowIt.Next(), i++){
			Row& r = row[i];

			// ��s�̏ꍇ�͍s�̂ݒǉ�����͍��Ȃ�
			string_iterator_pair str = rowIt.GetToken();
			if(str.empty())
				continue;

			// �񐔂𐔂���
			colIt.Set(str, delim, false);
			int ncol = 0;
			for(; !colIt.IsEnd(); colIt.Next())
				ncol++;
			r.col.resize(ncol);
			
			colIt.Set(str, delim, false);
			for(int j = 0; !colIt.IsEnd(); colIt.Next(), j++){
				r.col[j] = colIt.GetToken();
			}
		}

		// �����ɋ�s�������Ă���ꍇ�폜
		while(!row.empty() && row.back().col.empty())
			row.pop_back();
			
		return true;
	}

	/// �s��
	uint	NumRow(){
		return (uint)row.size();
	}

	/// i�s�ڂ̗�
	uint	NumCol(int i){
		return (uint)row[i].col.size();
	}

	/// ���x���i1�s�ڗv�f�j������̕�����ł�����T��
	int FindColumn(string label){
		if(NumRow() == 0)
			return -1;
		for(uint c = 0; c < NumCol(0); c++)
			if(Get<string>(0, c) == label)
				return c;
		return -1;
	}

	template<class T>
	T Get(string_iterator_pair str){
		T val;
		Converter::FromString(str, val);
		return val;
	}
	
	// �X�s�[�h�D��ŕ������ꉻ
	template<>
	int Get<int>(string_iterator_pair str){
		return atoi(to_string(str).c_str());
	}
	template<>
	uint Get<uint>(string_iterator_pair str){
		return (uint)atoi(to_string(str).c_str());
	}
	template<>
	float Get<float>(string_iterator_pair str){
		return (float)atof(to_string(str).c_str());
	}
	template<>
	double Get<double>(string_iterator_pair str){
		return (double)atof(to_string(str).c_str());
	}

	/// i�s, j��̗v�f���^�ϊ����ĕԂ�
	template<class T>
	T Get(uint i, uint j){
		if(i >= row.size())
			return T();
		if(j >= row[i].col.size())
			return T();

		return Get<T>(row[i].col[j]);
	}

};

}
