"""
A personal exercise for sqlite usage and strengthening my knowledge of
using git for development.

Plan is to slowly build the project more complex over time.
"""
import sqlite3 as sq

connection = sq.connect("phone_book_data.db")

cursor = connection.cursor()

