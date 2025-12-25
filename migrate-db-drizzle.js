import { neon } from '@neondatabase/serverless';
import { drizzle } from 'drizzle-orm/neon-http';
import { pgTable, serial, varchar, timestamp } from 'drizzle-orm/pg-core';
import { sql } from 'drizzle-orm';
import dotenv from 'dotenv';

dotenv.config();

// Define the same schema as in the auth server
const users = pgTable('users', {
  id: serial('id').primaryKey(),
  email: varchar('email', { length: 255 }).notNull().unique(),
  name: varchar('name', { length: 255 }),
  avatar: varchar('avatar', { length: 500 }),
  googleId: varchar('google_id', { length: 255 }).unique(), // For Google OAuth
  createdAt: timestamp('created_at').defaultNow().notNull(),
  updatedAt: timestamp('updated_at').defaultNow().notNull(),
});

// Create the database connection
const client = neon(process.env.DATABASE_URL);
const db = drizzle(client);

async function checkAndCreateTable() {
  try {
    console.log('Checking if users table exists...');

    // Try to query the table to see if it exists
    try {
      // This will fail if the table doesn't exist
      await db.execute(sql`SELECT 1 FROM users LIMIT 1;`);
      console.log('Users table exists.');
    } catch (error) {
      console.log('Users table does not exist. Creating it...');

      // Create the table manually using raw SQL
      await db.execute(sql`
        CREATE TABLE IF NOT EXISTS users (
          id SERIAL PRIMARY KEY,
          email VARCHAR(255) NOT NULL UNIQUE,
          name VARCHAR(255),
          avatar VARCHAR(500),
          google_id VARCHAR(255) UNIQUE,
          created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP NOT NULL,
          updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP NOT NULL
        );
      `);

      console.log('Users table created successfully.');
    }

    // Verify by checking the table structure
    const result = await db.execute(sql`
      SELECT column_name, data_type
      FROM information_schema.columns
      WHERE table_name = 'users'
      ORDER BY ordinal_position;
    `);

    console.log('Users table structure:');
    console.table(result);

    console.log('Database setup completed successfully!');

  } catch (error) {
    console.error('Error setting up database:', error);
  }
}

checkAndCreateTable();