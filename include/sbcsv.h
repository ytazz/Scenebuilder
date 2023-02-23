﻿#pragma once

#include <sbtypes.h>
#include <sbtokenizer.h>
#include <sbconverter.h>

namespace Scenebuilder{;

/* CSV読み込みクラス
	- ファイルコンテンツを文字列としてトークナイズし，値を取得時に毎回型変換する．
	- デリミタが連続した場合，１つの列として扱う
	- 改行が連続した場合，１つの行として扱う．
 */
class CsvReader{
public:
	struct Row{
		string line;
		vector<string_iterator_pair>	col;
	};
	vector<Row>	row;

	string contents;			///< contents string

	Tokenizer rowIt;
	Tokenizer colIt;

public:
	/// クリア
	void Clear(){
		contents.clear();
		row.clear();
	}

	/// 読み込む
	bool Read(const string& filename, const string& delim, bool compress = false){
		ifstream ifs;
		ifs.open(filename, ifstream::in | ifstream::binary);

		if(!ifs.is_open())
			return false;

		ifs.seekg(0, ifs.end);
		int len = (int)ifs.tellg();
		ifs.seekg(0, ifs.beg);

		// read all contents at once
		//contents.resize(len);
		//ifs.read(&contents[0], len);
		//ifs.close();
	
		// まず行数を数える
		//rowIt.Set(contents, "\n", compress);
		//int nrow = 0;
		//for(; !rowIt.IsEnd(); rowIt.Next())
		//	nrow++;
		int nrow = 0;
		string line;
		while(!ifs.eof()){
			std::getline(ifs, line);
			nrow++;
		}

		row.resize(nrow);

		//rowIt.Set(contents, "\n", compress);
		//for(int i = 0; !rowIt.IsEnd(); rowIt.Next(), i++){
		ifs.clear();
		ifs.seekg(0, ifs.beg);
		for(int i = 0; !ifs.eof(); i++){
			Row& r = row[i];
			
			// 空行の場合は行のみ追加し列は作らない
			//string_iterator_pair str = rowIt.GetToken();
			std::getline(ifs, r.line);
			if(r.line.empty())
				continue;

			// 列数を数える
			colIt.Set(r.line, delim, compress);
			int ncol = 0;
			for(; !colIt.IsEnd(); colIt.Next())
				ncol++;
			r.col.resize(ncol);
			
			colIt.Set(r.line, delim, compress);
			for(int j = 0; !colIt.IsEnd(); colIt.Next(), j++){
				r.col[j] = colIt.GetToken();
			}
		}

		// 末尾に空行が入っている場合削除
		while(!row.empty() && row.back().col.empty())
			row.pop_back();
			
		return true;
	}

	/// 行数
	uint	NumRow(){
		return (uint)row.size();
	}

	/// i行目の列数
	uint	NumCol(int i){
		return (uint)row[i].col.size();
	}

	/// ラベル（1行目要素）が所定の文字列である列を探す
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

	/// i行, j列の要素を型変換して返す
	template<class T>
	T Get(uint i, uint j){
		if(i >= row.size())
			return T();
		if(j >= row[i].col.size())
			return T();

		return Get<T>(row[i].col[j]);
	}

};

// スピード優先で部分特殊化
template<>
inline int CsvReader::Get<int>(string_iterator_pair str){
	return atoi(to_string(str).c_str());
}
template<>
inline uint CsvReader::Get<uint>(string_iterator_pair str){
	return (uint)atoi(to_string(str).c_str());
}
template<>
inline float CsvReader::Get<float>(string_iterator_pair str){
	return (float)atof(to_string(str).c_str());
}
template<>
inline double CsvReader::Get<double>(string_iterator_pair str){
	return (double)atof(to_string(str).c_str());
}

}
