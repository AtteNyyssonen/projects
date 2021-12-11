"""
Budget app project for FreeCodeAcademy Scientific Computing with Python course.
"""


class Category:

    def __init__(self, category):
        """
        initialize the needed attributes to class
        :param category: name of the category
        """
        self.category = category
        self.money = 0
        self.ledger = []

    def deposit(self, amount, description=False):
        """
        deposits money to ledger by append
        :param amount: deposit amount int
        :param description: deposit description str
        :return: None
        """
        self.money += amount

        if description:
            deposit_log = {"amount": amount, "description": description}
        else:
            deposit_log = {"amount": amount, "description": ""}
        self.ledger.append(deposit_log)

    def withdraw(self, wd_amount, wd_desc=False):
        """
        same as deposit but withdraws if the money is sufficient
        :param wd_amount: withdraw amount int
        :param wd_desc: withdraw description
        :return: True/False whether the withdraw was possible
        """

        if self.check_funds(wd_amount) is True:
            self.money -= wd_amount
            if wd_desc:
                wd_log = {"amount": (-1 * wd_amount), "description": wd_desc}
            else:
                wd_log = {"amount": (-1 * wd_amount), "description": ""}
            self.ledger.append(wd_log)

            return True
        else:
            return False

    def get_balance(self):
        """
        :return: self.money value
        """
        balance = self.money
        return balance

    def transfer(self, tf_amount, tf_category):

        if self.check_funds(tf_amount) is True:

            self.money -= tf_amount
            tf_category.money += tf_amount
            tf_log_self = {"amount": (-1 * tf_amount), "description":
                f"Transfer to {tf_category.category}"}
            tf_log_other = {"amount": tf_amount, "description":
                f"Transfer from {self.category}"}
            self.ledger.append(tf_log_self)
            tf_category.ledger.append(tf_log_other)

            return True

        else:
            return False

    def check_funds(self, to_wd_or_tf):
        if to_wd_or_tf <= self.money:
            return True
        else:
            return False

    def __str__(self):

        first_row = self.category.center(30, "*")

        all_logs = ""
        for logs in self.ledger:
            for value in logs.keys():
                amounts = logs["amount"]
            for descriptions in logs.values():
                desc = logs["description"]

                char_limit = desc[0:23]

            all_logs += f"{char_limit} {format(amounts, '.2f').rjust(len(first_row) - len(char_limit) - 1, ' ')}\n"
        current_balance = f"Total: {self.money:.2f}"

        return first_row + "\n" + f"{all_logs[:]}" + current_balance

    def total_wds_category(self):
        """
        calulates the total withdrawal for category
        :return: total withdraws per category
        """
        cg_wd = 0
        for amounts in self.ledger:
            if amounts["amount"] < 0:
                cg_wd += amounts["amount"]
        return cg_wd

def create_spend_chart(categories):
    """
    prints the spend chart for all the categories
    :param categories: all categories
    :return: print out
    """
    print_out = "Percentage spent by category\n"

    cat_wd = []

    for values in categories:
        total = values.total_wds_category()
        cat_wd.append(total)

    total_wd = sum(cat_wd)

    amount_percentage = []
    for value in cat_wd:
        percentage = round((value/total_wd)*100)
        amount_percentage.append(percentage)

    index = 100
    while index >= 0:

        category_spaces = " "
        for amounts in amount_percentage:
            if amounts >= index:
                category_spaces += "o  "
            else:
                category_spaces += "   "

        print_out += str(index).rjust(3) + "|" + category_spaces + "\n"
        index -= 10

    print_categories = []
    for dif_cat in categories:
        all_list = dif_cat.category
        print_categories.append(all_list)

    dash_lower = "-" + ("---"*len(categories))

    max_lenght =max(print_categories, key=len)

    x_line_for_print =""

    for x_position in range(len(max_lenght)):
        category_str = "     "
        for name_of_categories in print_categories:
            if x_position >= len(name_of_categories):
                category_str += "   "
            else:
                category_str += name_of_categories[x_position] + "  "

        if x_position != len(max_lenght)-1:
            category_str += "\n"


        x_line_for_print += category_str

    print_out += dash_lower.rjust(len(dash_lower)+4) + "\n" + x_line_for_print
    return print_out
