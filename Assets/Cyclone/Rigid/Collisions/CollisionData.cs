using System;
using System.Collections.Generic;

using Cyclone.Rigid.Constraints;

namespace Cyclone.Rigid.Collisions
{

    ///<summary>
    /// A helper structure that contains information for the detector to use
    /// in building its contact data.
    ///</summary>
    public class CollisionData
    {
        ///<summary> 
        /// Holds the contact array to write into. 
        ///</summary>
        public IList<RigidContact> Contacts;

        ///<summary> 
        /// Holds the maximum number of contacts the array can take. 
        ///</summary>
        public int ContactsLeft { get; private set; }

        ///<summary> 
        /// Holds the number of contacts found so far. 
        ///</summary>
        public int ContactCount { get; private set; }

        ///<summary> 
        /// Holds the friction value to write into any collisions. 
        ///</summary>
        public double Friction;

        ///<summary> 
        /// Holds the restitution value to write into any collisions. 
        ///</summary>
        public double Restitution;

        ///<summary>
        /// Holds the collision tolerance, even uncolliding objects this
        /// close should have collisions generated.
        ///</summary>
        public double Tolerance;

        ///<summary>
        /// Checks if there are more contacts.
        ///</summary>
        public bool HasMoreContacts()
        {
            return ContactsLeft > 0;
        }

        ///<summary>
        /// Checks if there are no more contacts.
        ///</summary>
        public bool NoMoreContacts()
        {
            return ContactsLeft <= 0;
        }

        ///<summary>
        /// Resets the data so that it has no used contacts recorded.
        ///</summary>
        public void Reset(int start)
        {
            ContactsLeft = Contacts.Count - start;
            ContactCount = start;
        }

        ///<summary>
        /// Notifies the data that the given number of contacts have
        /// been added.
        ///</summary>
        private void AddContacts(int count)
        {
            // Reduce the number of contacts remaining, add number used
            ContactsLeft -= count;
            ContactCount += count;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <returns></returns>
        public RigidContact GetContact()
        {
            if (NoMoreContacts())
                throw new InvalidOperationException("No more contacts.");

            var contact = Contacts[ContactCount];
            AddContacts(1);
            return contact;
        }
    }

}