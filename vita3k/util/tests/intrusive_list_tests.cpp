// Vita3K emulator project
// Copyright (C) 2026 Vita3K team
// SPDX-License-Identifier: GPL-2.0-or-later

#include <util/intrusive_list.h>

#include <gtest/gtest.h>

struct Item {
    int value;
    IntrusiveLink<Item> first;
    IntrusiveLink<Item> second;
};

using List = IntrusiveList<Item, &Item::first>;
using OtherList = IntrusiveList<Item, &Item::second>;

TEST(IntrusiveList, RemovalAndReinsertion) {
    Item a{ 1 }, b{ 2 }, c{ 3 }, d{ 4 };
    List list;
    EXPECT_TRUE(list.empty());
    list.push_back(a);
    EXPECT_TRUE(list.contains(a));
    EXPECT_EQ(list.size(), 1u);
    EXPECT_TRUE(list.remove(a));
    EXPECT_FALSE(list.remove(a));
    EXPECT_FALSE(a.first.linked());
    EXPECT_TRUE(list.empty());

    list.push_back(a);
    list.push_back(b);
    list.push_back(c);
    list.push_back(d);
    auto it = list.begin();
    ++it;
    EXPECT_TRUE(list.remove(*it++));
    EXPECT_EQ(it->value, 3);
    EXPECT_TRUE(list.remove(a));
    EXPECT_TRUE(list.remove(d));
    EXPECT_EQ(list.size(), 1u);
    EXPECT_EQ(list.front().value, 3);
    list.insert(list.begin(), b);
    list.push_back(a);
    const List &view = list;
    auto current = view.begin();
    EXPECT_EQ((current++)->value, 2);
    EXPECT_EQ((current++)->value, 3);
    EXPECT_EQ((current++)->value, 1);
    EXPECT_EQ(current, view.end());
    EXPECT_TRUE(list.remove(b));
    EXPECT_TRUE(list.remove(c));
    EXPECT_TRUE(list.remove(a));
    EXPECT_EQ(list.size(), 0u);
}

TEST(IntrusiveList, IndependentHooksAndMembership) {
    Item a{ 1 }, b{ 2 };
    List first, another;
    OtherList second;
    first.push_back(a);
    another.push_back(b);
    second.push_back(a);
    second.push_back(b);
    EXPECT_TRUE(first.contains(a));
    EXPECT_FALSE(another.contains(a));
    EXPECT_TRUE(second.contains(a));
    EXPECT_TRUE(first.remove(a));
    EXPECT_FALSE(a.first.linked());
    EXPECT_TRUE(a.second.linked());
    EXPECT_TRUE(second.remove(a));
    EXPECT_TRUE(second.remove(b));
    EXPECT_TRUE(another.remove(b));
}
