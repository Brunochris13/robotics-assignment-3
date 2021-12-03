class Restaurant():
    """
    Note:
        Mainly methods should be used to update restaurnat attributes
        because multiple robots may connect to the same restaurant and
        updates may be pushed to external databases.
    """
    
    def __init__(self):
        self.entrance = None # read from file/db (position)
        self.kitchen = None # read from file/db (position)
        self.centre = None # read from file/db (position)
        self.tables = None # read from file/db (list of positions)
        self.order_history = []
    

    def get_menu(self, n18=False):
        """Gets the restaurant menu with food names and prices.
        
        Args:
            n18 (bool): Whether to _only_ return products marked 18+

        Returns:
            (dict):
        """
        pass


    def get_table_by_id(self, table_id):
        """Gets the table object by its ID.

        Args:
            table_id (int): The ID of the requested table

        Returns:
            (Table): A table object whose ID is `table_id`.
        """
        table_ids = [table.id for table in self.tables]
        return self.tables[table_ids.index(table_id)]


    def get_tables_by_age_group(self, age_group):
        """Gets the list of tables that belong to the given age group.

        Args:
            age_group (range):
        
        Returns:
            (list(Table)):
        """
        pass


    def set_occupied_table(self, table_id):
        """Sets the table as occupied (e.g. in database)

        Args:
            table_id (int):
        """
        pass


    def get_available_tables(self, num_people=1):
        """Gets the available tables based on the number of people.

        Returns:
            (list(Table)):
        """
        pass


    def get_food_ready(self):
        """Gets the list of order IDs for which the food is ready.

        Returns:
            (list(int)):
        """
        pass


    def get_bill_ready(self):
        """Gets the list of order IDs for which the bill is ready.

        Returns:
            (list(int)):
        """
        pass
    
    
    def new_customer_exists(self):
        """Checks if there's any new customers at the entrance.

        Returns:
            (bool):
        """
        pass


    def update(self, orders):
        """Updates the state of the restaurant.

        For example, reads a .txt file to see if someone has finished eating.
        Or subscribes to a topic to check for any new updates.
        """
        pass