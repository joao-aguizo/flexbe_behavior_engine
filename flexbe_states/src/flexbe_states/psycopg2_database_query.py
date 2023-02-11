#!/usr/bin/env python

import re

# DB modules
import psycopg2
import psycopg2.sql as sql
import psycopg2.extras

from flexbe_core import EventState, Logger


class Psycopg2DatabaseQuery(EventState):
	'''
	Reads a custom query from a custom database connection. Relies on the psycopg2 library. Allows dynamic SQL queries defining (e.g. SELECT value FROM {storage} where collection=%s) (https://www.psycopg.org/docs/usage.html#passing-parameters-to-sql-queries).

	-- dsn string	Dsn connection string.
	-- query string	Custom query to fetch data.

	># data2write 		string 	Fetched data.

	<= succeeded 		Successfully executed query.
	<= failed		Failed executing query.
	<= connection_error		Failed connecting to database.

	'''

	def __init__(self, dsn, query, input_keys, fetch = True):
		# See example_state.py for basic explanations.
		super(Psycopg2DatabaseQuery, self).__init__(outcomes = ['succeeded', 'failed', 'connection_error'],
												 input_keys = input_keys,
												 output_keys = ['fetched_result'])

		self._conn = None
		self._query = query
		self._dsn = dsn
		self._input_keys = input_keys
		self._fetch = fetch
		self._error = False


	def execute(self, userdata):
		'''
		Access the database and executes defined query
		'''
		# Check if the connection to the database failed
		if self._error:
			return 'connection_error'

		# Access the database and gather all the desired information
		userdata.fetched_result = None
		try:
			with self._conn.cursor() as cur:

				params = None

				# Find all identifiers with a regex expression
				identifiers = re.findall("\{(..*?)\}", self._query)
				# Logger.logwarn(str(identifiers))

				# Get the list of params, removing the identifiers (only if there are any)
				if self._input_keys:
					params = [userdata[key] for key in self._input_keys if key not in identifiers]

				# Associate each identifier with its respective value received through available in userdata
				args_dict = {}
				for key in identifiers:
					args_dict[key] = sql.Identifier(userdata[key]) if "literal" not in key else sql.Literal(userdata[key])
				# Logger.logwarn(args_dict)
				
				# Format the SQL statement using the string format method
				f_query = sql.SQL(self._query).format(**args_dict)
				# Logger.logwarn(str(f_query))

				cur.execute(f_query, params)
				self._conn.commit()

				data = []
				if self._fetch:
					data = cur.fetchall()

				userdata.fetched_result = data

				return 'succeeded'

		except Exception as e:
			Logger.logwarn('Failed to fetch data from database:\n%s' % str(e))
			return 'failed'


	def on_enter(self, userdata):
		# Opens connection to the database when entering the state

		self._error = False
		try:
			self._conn = psycopg2.connect(self._dsn)
		except Exception as e:
			# Using a linebreak before appending the error log enables the operator to collapse details in the GUI.
			Logger.logwarn('Failed to connect to database:\n%s' % str(e))
			self._error = True


	def on_exit(self, userdata):
		# Make sure that the connection to the database is closed when leaving the state.

		# Checks if connection is alive
		if (self._conn != None and self._conn.closed == 0):
			self._conn.close()
			Logger.loginfo('Connection to the database was closed.')