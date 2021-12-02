#!/usr/bin/env python

from enum import Enum, auto


class OrderStatus(Enum):
    WAITING_SEAT = auto()
    WAITING_FOOD = auto()
    EATING = auto()
    WAITING_BILL = auto()
    SUCCEEDED = auto()
    CANCELED = auto()


class Order():
    next_id = 0

    def __init__(self, table=None, people=None, contents=None):
        self.id = Order.next_id
        self.status = OrderStatus.WAITING_SEAT
        self.table = table
        self.people = people
        self.contents = contents

        Order.next_id += 1
        