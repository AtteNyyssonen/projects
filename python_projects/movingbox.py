"""
COMP.CS.100 Ohjelmointi 1 / Programming 1
Name:       Atte Nyyssönen
Email:      atte.nyyssonen@tuni.fi

Program where you can store data of different moving boxes and their items.
Then with the name of a certain moving box you can add items or list the
items out, transferring items between boxes is also possible. You can also
list out the data of all the boxes or search/count items in the boxes.

Everything in this program is written in English because I didn't want to mix
two languages together. The feedback can be given in Finnish and I will choose
that option in plussa, just wanted to write this here so that
it would be clearer.
"""


class MovingBox:
    """
    A class for keeping track of the contents of
    a moving box. Items can be added to, removed from,
    itemized (i.e. listed), and searched from a moving box.
    The number of items in a box can also be queried
    and items can be transferred between two boxes.
    """

    def __init__(self, box_name):
        """
        Creates a moving box with attributes name and dict as a item list.
        The dictionary will hold item names as keys and the amounts of that
        particular item as the values.
        :param box_name: User given name for the moving box
        """

        self.__name = box_name
        self.__box_items = {}

    def add_item(self, item_name, item_count):
        """
        Adds the user inputted item into the directed moving box.
        Checks to see if the item already exists in the dict or doesn't.
        If it already exists we just add the amount to the prior amount.

        :param item_name: str name of the item.
        :param item_count: int amount of the above mentioned item.
        :return: updated box_items dictionary with the item and the amount.
        """

        if item_name not in self.__box_items:
            self.__box_items[item_name] = item_count
        elif item_name in self.__box_items:
            self.__box_items[item_name] += item_count

        return self.__box_items

    def list_content(self):
        """
        Prints out the sum of all items, then lists the items in alphabetical
        order indented with specific names and amounts.
        First goes through the inputted boxes dictionary values and sums them
        together. Then Prints the initial message of the boxes name and
        the total items. After the initial message prints the amount
        of items and the item names in alphabetical order.

        :return: None.
        """

        amounts = self.__box_items.values()
        total = sum(amounts)

        print(f"Box \"{self.__name}\" contains {total} items.")

        for item in sorted(self.__box_items):
            print(f"{self.__box_items[item]:3.0f} {item}")

    def remove_item(self, item_name, item_count):
        """
        Removes the users wanted amount of a certain item from a box.
        First checks to see if the wanted item is in the box.
        Then checks to see that it's a rational deduction of that
        certain item. If it is then subtracting the wanted amount.
        Next if the certain item value drops to zero we will delete it,
        because that item no longer exists in the box.

        :param item_name: str user inputted name of the wanted box.
        :param item_count: int the amount of that particular item.
        :return: boolean True to tell the calling function removal happened.
        """

        if item_name in self.__box_items:
            if item_count <= self.__box_items[item_name]:
                self.__box_items[item_name] -= item_count

                if self.__box_items[item_name] == 0:
                    del self.__box_items[item_name]
                return True

    def transfer_item(self, target_box, item_name, item_count):
        """
        Transfers an user inputted item from a box to another.
        First check to see if the transfer item exists.
        Then check to make sure we aren't moving items that don't exist.
        If the item amount from the transfer box drops to zero we will
        delete the item value.
        Next if the item doesn't exist in the target box we will add the
        item name as a key and the count as the value.
        If the key value pair already exists we will just add the count to
        the existing value.

        :param target_box: str user inputted target box for the item.
        :param item_name: str name of the item wanted to transfer.
        :param item_count: int the amount of the item wanted to transfer.
        :return: boolean True to tell function the transfer was successful.
        """

        if item_name in self.__box_items:
            if item_count <= self.__box_items[item_name]:
                self.__box_items[item_name] -= item_count

                if item_name not in target_box.__box_items:
                    target_box.__box_items[item_name] = item_count
                elif item_name in target_box.__box_items:
                    target_box.__box_items[item_name] += item_count

                if self.__box_items[item_name] == 0:
                    del self.__box_items[item_name]
                return True

    def search_item(self, item_name):
        """
        Searches the wanted item from the boxes. This method is also used
        for the counting of certain items in the boxes. The method is called
        in a loop that iterates through all the boxes.

        :param item_name: str the user inputted item that is searched.
        :return: str value of the inputted item if it exists if it
        doesn't exist returns a false to get rid of Nonetype errors.
        """
        
        if item_name in self.__box_items:
            return self.__box_items[item_name]
        else:
            return False


def convert_str_to_int(item_count):
    """
    Used call to change item count str to int so that
    in the methods item counts can be added, removed or summed.
    I think this function was lost in the making of this exercise and
    wasn't pre-included in the downloadable file.
    :param item_count: str of the amount of items
    :return: int of the amount of items
    """

    return int(item_count)


##### DON'T CHANGE ANYTHING AFTER THIS LINE ##########################



# NOTE!
# The following functions have a second parameter whose type is always
# a list containing strings, even in those cases when the list is empty.
# The reason for this is that the command interface
# in the main function was kept very simple: different commands
# require a different number of words of user input. The main function
# just stores all the extra words given after the actual command word in
# the user input line in single list without worrying
# if they are correct or not. This list is then passed as
# a parameter to the actual command functions below. It is their
# job to check that the words in the list makes sense in the context
# of the command in question.


def newbox(all_boxes, list_of_additional_info):
    """
    Creates a new moving box named after the first element in
    the parameter *list_of_additional_info*. The new box is stored
    in the dictionary *all_boxes* using the box's name as the key.
    If *all_boxes* already contains a box with the same name,
    it will be removed and replaced with the new box.

    :param all_boxes: A dictionary containing moving box objects.
           The name of the box is used as the key.
    :param list_of_additional_info: A list containing a single element:
           the name of the moving box (str) to be created.
    :return: None
    """

    if len(list_of_additional_info) != 1:
        print("Error: wrong number of initial data: can't create a new box.")
        return

    box_name = list_of_additional_info[0]
    all_boxes[box_name] = MovingBox(box_name)


def add_to_box(all_boxes, list_of_additional_info):
    """
    Adds a new item into a box stored in the dictionary *all_boxes*.
    If there is no box named by the first element of the list
    *list_of_additional_info*, then an error message is shown.

    :param all_boxes: A dictionary containing moving box objects.
           The name of the box is used as the key.
    :param list_of_additional_info: A list containing 3 elements:
           the name of the box (str),
           the name of the item (str), and
           the number of items to be added (str).
    :return: None
    """

    if len(list_of_additional_info) != 3:
        print("Error: wrong number of elements: can't add into a box.")
        return

    box_name, item_name, item_count = list_of_additional_info
    item_count = convert_str_to_int(item_count)

    if item_count is None:
        print("Error: not a number: can't add to a box.")
        return

    if box_name not in all_boxes:
        print("Error: box does not exist: can't add to a box.")
        return

    all_boxes[box_name].add_item(item_name, item_count)


def remove_from_box(all_boxes, list_of_additional_info):
    """
    Removes a given amount of named items from a box stored
    in the dictionary *all_boxes*.
    If there is no box named by the first element of the list
    *list_of_additional_info*, then an error message is displayed.
    If the box exits but doesn't contain the named item,
    then an error message is also displayed.
    If the parameter giving the number of items to be removed
    is larger than what's available in the box, then again
    an error message is displayed.

    :param all_boxes: A dictionary containing moving box objects.
           The name of the box is used as the key.
    :param list_of_additional_info: A list containing 3 elements:
           the name of the box (str),
           the name of the item (str), and
           the number of items to be removed (str).
    :return: None
    """

    if len(list_of_additional_info) != 3:
        print("Error: wrong number of elements: can't remove from a box.")
        return

    box_name, item_name, item_count = list_of_additional_info
    item_count = convert_str_to_int(item_count)

    if item_count is None:
        print("Error: not a number: can't remove from a box.")
        return

    if box_name not in all_boxes:
        print("Error: box does not exist: can't remove from a box.")
        return

    if not all_boxes[box_name].remove_item(item_name, item_count):
        print("Error: removing an item from a box failed.")


def transfer_from_box_to_another_box(all_boxes, list_of_additional_info):
    """
    Transfers some amount of named items from one moving box to into another.

    :param all_boxes: all_boxes: A dictionary containing moving box objects.
           The name of the box is used as the key.
    :param list_of_additional_info: A list containing 4 elements:
           the name of the source box (str),
           the name of the target box (str),
           the name of the item to move (str), and
           the number of items to move (str)
    :return: None
    """

    if len(list_of_additional_info) != 4:
        print("Error: wrong number of elements: can't transfer items.")
        return

    source_name, target_name, item_name, item_count = list_of_additional_info
    item_count = convert_str_to_int(item_count)

    if item_count is None:
        print("Error: not a number: can't transfer items.")
        return

    if source_name not in all_boxes or target_name not in all_boxes:
        print("Error: unknown box name: can't transfer items.")

    if not all_boxes[source_name].transfer_item(all_boxes[target_name], item_name, item_count):
        print("Error: item transfer failed for some reason.")


def list_box_content(all_boxes, list_of_additional_info):
    """
    Displays the contents of a single box in *all_boxes*
    dictionary. If there is no box named by the first element of the list
    *list_of_additional_info*, then an error message is displayed.

    :param all_boxes: A dictionary containing moving box objects.
           The name of the box is used as the key.
    :param list_of_additional_info: A list containing 1 element:
           the name of the box (str)
    :return: None
    """

    if len(list_of_additional_info) != 1:
        print("Error: wrong number of elements: can't list contents.")
        return

    box_name = list_of_additional_info[0]

    if box_name not in all_boxes:
        print("Error: box does not exist: can't list content.")
        return

    all_boxes[box_name].list_content()


def list_all_boxes(all_boxes, list_of_additional_info):
    """
    Prints the contents of all boxes on the screen.
    The boxes are printed in an alphabetical order.
    The content of each individual box is printed
    in the same format as it is done in the function
    *list_box_content* above.

    :param all_boxes: A dictionary containing moving box objects.
           The name of the box is used as the key.
    :param list_of_additional_info: An empty list since this function
           does not require any other data than *all_boxes*.
    :return: None
    """

    if len(list_of_additional_info) != 0:
        print("Error: wrong number of elements: boxes not listed.")
        return

    for box_name in sorted(all_boxes):
        list_box_content(all_boxes, [box_name])


def search_all_boxes(all_boxes, list_of_additional_info):
    """
    Searches a particular item in all of the boxes and
    and prints which boxes contained the item and how
    many of it there was. The boxes in *all_boxes* are
    checked in an alphabetical order. If no items are found,
    the function prints nothing.

    :param all_boxes: A dictionary containing moving box objects.
           The name of the box is used as the key.
    :param list_of_additional_info: A list containing one element (str)
           which is the name of the item the function should search for.
    :return: None
    """

    if len(list_of_additional_info) != 1:
        print("Error: wrong number of elements: can't search boxes.")
        return

    item_name = list_of_additional_info[0]

    for box_name in sorted(all_boxes):
        item_count = all_boxes[box_name].search_item(item_name)

        if item_count > 0:
            print(f"Box {box_name}: {item_count}")


def count_specific_items(all_boxes, list_of_additional_info):
    """
    Counts and prints the number of a particular items
    inside all of the boxes contained in *all_boxes*.
    Zero is printed if there are none.

    :param all_boxes: A dictionary containing moving box objects.
           The name of the box is used as the key.
    :param list_of_additional_info: A list containing one element (str)
           which is the name of the item the function should count.
    :return: None
    """

    if len(list_of_additional_info) != 1:
        print("Error: wrong number of elements: can't count items.")
        return

    item_name = list_of_additional_info[0]

    item_count = 0
    for box in all_boxes.values():
        item_count += box.search_item(item_name)

    print(f"There are total {item_count} {item_name}(s) in the boxes.")


def main():
    """
    In this project *main* function contains the top level
    part of the program's user interface.

    **Do not modify the main function in any way: the automated
    tests rely it to behave in a pre-defined way.**
    """
    boxes = {}

    while True:
        command_line = input("next command> ").strip()
        if command_line == "":
            break

        command_words = command_line.split()
        first_word = command_words[0]
        list_of_other_words = command_words[1:]

        if first_word == "quit":
            break

        elif first_word == "newbox":
            newbox(boxes, list_of_other_words)

        elif first_word == "add":
            add_to_box(boxes, list_of_other_words)

        elif first_word == "remove":
            remove_from_box(boxes, list_of_other_words)

        elif first_word == "transfer":
            transfer_from_box_to_another_box(boxes, list_of_other_words)

        elif first_word == "list":
            list_box_content(boxes, list_of_other_words)

        elif first_word == "list_all":
            list_all_boxes(boxes, list_of_other_words)

        elif first_word == "search":
            search_all_boxes(boxes, list_of_other_words)

        elif first_word == "count":
            count_specific_items(boxes, list_of_other_words)

        else:
            print(f"Error: \"{first_word}\" is not a recognized command!")


if __name__ == "__main__":
    main()
