#include <iostream>
#include <memory>
using namespace std;

int main()
{
    auto p1 = make_shared<string>("this is a shared pointer");                            // 对应类的共享指针 shared_ptr<string>,写成 auto
    cout << "p1的引用计数：" << p1.use_count() << "，指向内存地址：" << p1.get() << endl; // 1

    auto p2 = p1;
    cout << "p1的引用计数：" << p1.use_count() << "，指向内存地址：" << p1.get() << endl; // 2
    cout << "p2的引用计数：" << p2.use_count() << "，指向内存地址：" << p2.get() << endl; // 2

    p1.reset();                                                                           // 释放p1指向的内存
    cout << "p1的引用计数：" << p1.use_count() << "，指向内存地址：" << p1.get() << endl; // 0
    cout << "p2的引用计数：" << p2.use_count() << "，指向内存地址：" << p2.get() << endl; // 1

    cout << "p2的值：" << p2->c_str() << endl; // 访问p2指向的字符串内容

    return 0;
}
