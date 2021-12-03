#!/usr/bin/env python

from enum import Enum, auto


class OrderStatus(Enum):
    WAITING_SEAT = auto()
    WAITING_FOOD = auto()
    PROCESS_FOOD = auto()
    WAITING_BILL = auto()
    FINISHED = auto()
    CANCELED = auto()


class Order():
    next_id = 0

    def __init__(self, table=None, people=None):
        self.id = Order.next_id
        self.status = OrderStatus.WAITING_SEAT
        self.table = table
        self.people = people
        self.contents = []

        Order.next_id += 1
        