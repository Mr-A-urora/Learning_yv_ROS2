#include <iostream>
#include <functional>
using namespace std;

// 自由函数
void save_with_free_fun(const string &file_name)
{
    cout << "自由函数：" << file_name << endl;
}

// 成员函数
class FileSave
{
private:

public:
    FileSave() = default;
    ~FileSave() = default;

    void save_with_member_fun(const string &file_name)
    {
        cout << "成员函数：" << file_name << endl;
    };
};

int main()
{
    FileSave file_save;

    // Lambda函数
    auto save_with_lambda_fun = [](const string &file_name)
    {
        cout << "Lambda函数：" << file_name << endl;
    };

    /*
    save_with_free_fun("file.txt");
    file_save.save_with_member_fun("file.txt");
    save_with_lambda_fun("file.txt");
    */

    // 使用std::function包装函数对象
    function<void(const string &)> save1 = save_with_free_fun;
    function<void(const string &)> save2 = save_with_lambda_fun;

    //成员函数放入包装器
    function<void(const string &)> save3 = bind(&FileSave::save_with_member_fun, &file_save, placeholders::_1);

    // 统一的调用的方法
    save1("file.txt");
    save2("file.txt");
    save3("file.txt");

    return 0;
}
