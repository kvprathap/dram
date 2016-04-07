#ifndef __SCHEDULER_H
#define __SCHEDULER_H

#include "DRAM.h"
#include "Request.h"
#include "Controller.h"
#include <vector>
#include <map>
#include <list>
#include <functional>
#include <cassert>
#include <stdarg.h>

using namespace std;

namespace ramulator
{

template <typename T>
class Controller;

template <typename T>
class Scheduler
{
public:
    Controller<T>* ctrl;

    enum class Type {
        FCFS, FRFCFS, FRFCFS_Cap, FRFCFS_PriorHit, MEDUSA_FRFCFS_PriorHit, MEDUSA_NO_SWITCH_FRFCFS_PriorHit, MAX
    //} type = Type::FRFCFS_PriorHit;
    } type = Type::MEDUSA_FRFCFS_PriorHit;

    long cap = 16;
    //Bits corresponding to reserved banks are set.
    uint8_t reservedBankMask = 0xf;
    //The corresponding bits of serviced banks
    //are cleared within a round.
    uint8_t rrBankMask = reservedBankMask;
    //found a ready request request to reserved bank.
    bool found_ready_reserved_request;
    //found a not-ready request request to reserved bank
    bool found_reserved_request;


    Scheduler(Controller<T>* ctrl) : ctrl(ctrl) {}

    // Checks if there are pending requests to
    // reserved banks in the queue.
    bool isRequestToReservedBank(list<Request>& q) {
        for (auto itr = q.begin() ; itr != q.end() ; ++itr) {
            if ((reservedBankMask) & (0x01 << itr->addr_vec[int (T::Level::Bank)]))
                return true;
        }
        return false;
    }


    list<Request>::iterator get_head(list<Request>& q)
    {
      // TODO make the decision at compile time
      if (type != Type::FRFCFS_PriorHit && type != Type::MEDUSA_FRFCFS_PriorHit && type != Type::MEDUSA_NO_SWITCH_FRFCFS_PriorHit) {
        if (!q.size())
            return q.end();

        auto head = q.begin();
        for (auto itr = next(q.begin(), 1); itr != q.end(); itr++)
            head = compare[int(type)](head, itr);

        return head;
      //MEDUSA: Round-robin scheduling
      } else if (type == Type::MEDUSA_FRFCFS_PriorHit || type == Type::MEDUSA_NO_SWITCH_FRFCFS_PriorHit) {
        if(this->ctrl->write_mode == true)
            goto frfcfs;
        found_ready_reserved_request = false;
        found_reserved_request = false;
        //Resets the rrBankMask to begin a new round.
        if (!rrBankMask)
            rrBankMask = reservedBankMask;
        //printf("Value of rrBankMask = 0x%lx\n", rrBankMask);
        //printf("MEDUSA\n");
        if (!q.size())
            return q.end();
        auto head = q.begin();
        //find the request to reserved bank which is not
        //serviced yet in the current round.
        for (auto itr = next(q.begin(), 1); itr != q.end(); itr++) {
            head = compare[int(type)](head, itr);
        }
        if (found_ready_reserved_request || found_reserved_request) {
            //printf("MEDUSA:Selected Bank: %d\n", head->addr_vec[int (T::Level::Bank)]);
            return head;
        }
        else {
            //When a round is done, check if there are
            //more requests to the reserved bank that is
            //already served in the current round.
            if (isRequestToReservedBank(q)) {
                //start new round.
                rrBankMask = reservedBankMask;
                for (auto itr = next(q.begin(), 1); itr != q.end(); itr++) {
                    head = compare[int(type)](head, itr);
                }
                if (found_ready_reserved_request || found_reserved_request) {
                    return head;
                }
            }
            // No more requests to reserved banks.
            // switch to fr-fcfs to schedule requests
            // to shared banks.
            goto frfcfs;
        }
      }
      else {
frfcfs:
        //printf("FR-FCFS\n");
        if (!q.size())
            return q.end();

        auto head = q.begin();
        for (auto itr = next(q.begin(), 1); itr != q.end(); itr++) {
            head = compare[int(Type::FRFCFS_PriorHit)](head, itr);
        }

        if (this->ctrl->is_ready(head) && this->ctrl->is_row_hit(head)) {
          //printf("FR-FCFS:Selected Bank: %d\n", head->addr_vec[int (T::Level::Bank)]);
          return head;
        }

        // prepare a list of hit request
        vector<vector<int>> hit_reqs;
        for (auto itr = q.begin() ; itr != q.end() ; ++itr) {
          if (this->ctrl->is_row_hit(itr)) {
            auto begin = itr->addr_vec.begin();
            // TODO Here it assumes all DRAM standards use PRE to close a row
            // It's better to make it more general.
            auto end = begin + int(ctrl->channel->spec->scope[int(T::Command::PRE)]) + 1;
            vector<int> rowgroup(begin, end); // bank or subarray
            hit_reqs.push_back(rowgroup);
          }
        }
        // if we can't find proper request, we need to return q.end(),
        // so that no command will be scheduled
        head = q.end();
        for (auto itr = q.begin(); itr != q.end(); itr++) {
          bool violate_hit = false;
          if ((!this->ctrl->is_row_hit(itr)) && this->ctrl->is_row_open(itr)) {
            // so the next instruction to be scheduled is PRE, might violate hit
            auto begin = itr->addr_vec.begin();
            // TODO Here it assumes all DRAM standards use PRE to close a row
            // It's better to make it more general.
            auto end = begin + int(ctrl->channel->spec->scope[int(T::Command::PRE)]) + 1;
            vector<int> rowgroup(begin, end); // bank or subarray
            for (const auto& hit_req_rowgroup : hit_reqs) {
              if (rowgroup == hit_req_rowgroup) {
                  violate_hit = true;
                  break;
              }
            }
          }
          if (violate_hit) {
            continue;
          }
          // If it comes here, that means it won't violate any hit request
          if (head == q.end()) {
            head = itr;
          } else {
            head = compare[int(Type::FRFCFS)](head, itr);
          }
        }

        return head;
      }
    }

private:
    typedef list<Request>::iterator ReqIter;
    function<ReqIter(ReqIter, ReqIter)> compare[int(Type::MAX)] = {
        // FCFS
        [this] (ReqIter req1, ReqIter req2) {
            if (req1->arrive <= req2->arrive) return req1;
            return req2;},

        // FRFCFS
        [this] (ReqIter req1, ReqIter req2) {
            bool ready1 = this->ctrl->is_ready(req1);
            bool ready2 = this->ctrl->is_ready(req2);

            if (ready1 ^ ready2) {
                if (ready1) return req1;
                return req2;
            }

            if (req1->arrive <= req2->arrive) return req1;
            return req2;},

        // FRFCFS_CAP
        [this] (ReqIter req1, ReqIter req2) {
            bool ready1 = this->ctrl->is_ready(req1);
            bool ready2 = this->ctrl->is_ready(req2);

            ready1 = ready1 && (this->ctrl->rowtable->get_hits(req1->addr_vec) <= this->cap);
            ready2 = ready2 && (this->ctrl->rowtable->get_hits(req2->addr_vec) <= this->cap);

            if (ready1 ^ ready2) {
                if (ready1) return req1;
                return req2;
            }

            if (req1->arrive <= req2->arrive) return req1;
            return req2;},
        // FRFCFS_PriorHit
        [this] (ReqIter req1, ReqIter req2) {
            bool ready1 = this->ctrl->is_ready(req1) && this->ctrl->is_row_hit(req1);
            bool ready2 = this->ctrl->is_ready(req2) && this->ctrl->is_row_hit(req2);

            if (ready1 ^ ready2) {
                if (ready1) return req1;
                return req2;
            }

            if (req1->arrive <= req2->arrive) return req1;
            return req2;},
        // MEDUSA_FRFCFS_PriorHit,MEDUSA_NO_SWITCH_FRFCFS_PriorHit
        [this] (ReqIter req1, ReqIter req2) {

            // find the first come ready request to reserved bank.
            bool ready1 = (this->ctrl->is_ready(req1)) && ((rrBankMask) & (0x01 << req1->addr_vec[int (T::Level::Bank)]));
            bool ready2 = (this->ctrl->is_ready(req2)) && ((rrBankMask) & (0x01 << req2->addr_vec[int (T::Level::Bank)]));
            //printf("Req1 Bank: %d Req2 Bank: %d\n", req1->addr_vec[int (T::Level::Bank)], req2->addr_vec[int (T::Level::Bank)]);

            if (ready1 || ready2) {
                this->found_ready_reserved_request = true;
            }
            if (ready1 ^ ready2) {
                if (ready1) return req1;
                return req2;
            }
            if (ready1 && ready2) {
                if (req1->arrive <= req2->arrive)
                    return req1;
                return req2;}


            // if we cannot find the ready request.
            // find the first come non-ready request to reserved bank.
            ready1 = ((rrBankMask) & (0x01 << req1->addr_vec[int (T::Level::Bank)]));
            ready2 = ((rrBankMask) & (0x01 << req2->addr_vec[int (T::Level::Bank)]));
            if (ready1 || ready2) {
                this->found_reserved_request = true;
            }

            if (ready1 ^ ready2) {
                if (ready1) return req1;
                return req2;
            }

            if (ready1 && ready2) {
                if (req1->arrive <= req2->arrive)
                    return req1;
                return req2;
            }
            //printf("flag1 %d, flag2 %d\n",this->found_ready_reserved_request, this->found_reserved_request);
            if (req1->arrive <= req2->arrive) return req1;
            return req2;
        }
    };
};


template <typename T>
class RowPolicy
{
public:
    Controller<T>* ctrl;

    enum class Type {
        Closed, Opened, Timeout, MAX
    } type = Type::Opened;

    int timeout = 50;

    RowPolicy(Controller<T>* ctrl) : ctrl(ctrl) {}

    vector<int> get_victim(typename T::Command cmd)
    {
        return policy[int(type)](cmd);
    }

private:
    function<vector<int>(typename T::Command)> policy[int(Type::MAX)] = {
        // Closed
        [this] (typename T::Command cmd) -> vector<int> {
            for (auto& kv : this->ctrl->rowtable->table) {
                if (!this->ctrl->is_ready(cmd, kv.first))
                    continue;
                return kv.first;
            }
            return vector<int>();},

        // Opened
        [this] (typename T::Command cmd) {
            return vector<int>();},

        // Timeout
        [this] (typename T::Command cmd) -> vector<int> {
            for (auto& kv : this->ctrl->rowtable->table) {
                auto& entry = kv.second;
                if (this->ctrl->clk - entry.timestamp < timeout)
                    continue;
                if (!this->ctrl->is_ready(cmd, kv.first))
                    continue;
                return kv.first;
            }
            return vector<int>();}
    };

};


template <typename T>
class RowTable
{
public:
    Controller<T>* ctrl;

    struct Entry {
        int row;
        int hits;
        long timestamp;
    };

    map<vector<int>, Entry> table;

    RowTable(Controller<T>* ctrl) : ctrl(ctrl) {}

    void update(typename T::Command cmd, const vector<int>& addr_vec, long clk)
    {
        auto begin = addr_vec.begin();
        auto end = begin + int(T::Level::Row);
        vector<int> rowgroup(begin, end); // bank or subarray
        int row = *end;

        T* spec = ctrl->channel->spec;

        if (spec->is_opening(cmd))
            table.insert({rowgroup, {row, 0, clk}});

        if (spec->is_accessing(cmd)) {
            // we are accessing a row -- update its entry
            auto match = table.find(rowgroup);
            assert(match != table.end());
            assert(match->second.row == row);
            match->second.hits++;
            match->second.timestamp = clk;
        } /* accessing */

        if (spec->is_closing(cmd)) {
          // we are closing one or more rows -- remove their entries
          int n_rm = 0;
          int scope = int(spec->scope[int(cmd)]);
          for (auto it = table.begin(); it != table.end();) {
            if (equal(begin, begin + scope + 1, it->first.begin())) {
              n_rm++;
              it = table.erase(it);
            }
            else
              it++;
          }
          assert(n_rm > 0);
        } /* closing */
    }

    int get_hits(vector<int>& addr_vec)
    {
        auto begin = addr_vec.begin();
        auto end = begin + int(T::Level::Row);

        vector<int> rowgroup(begin, end);
        int row = *end;

        auto itr = table.find(rowgroup);
        auto entry = table[rowgroup];
        if (itr == table.end() || entry.row != row)
            return 0;

        return entry.hits;
    }
};

} /*namespace ramulator*/

#endif /*__SCHEDULER_H*/
