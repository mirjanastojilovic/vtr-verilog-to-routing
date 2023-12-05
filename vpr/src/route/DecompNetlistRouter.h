#pragma once

/** @file Parallel and net-decomposing case for NetlistRouter. Works like
 * \see ParallelNetlistRouter, but tries to "decompose" nets and assign them to
 * the next level of the partition tree where possible. */
#include "netlist_routers.h"

#include <tbb/task_group.h>

template<typename HeapType>
class DecompNetlistRouter : public NetlistRouter {
  public:
    DecompNetlistRouter(
        const Netlist<>& net_list,
        const RouterLookahead* router_lookahead,
        const t_router_opts& router_opts,
        CBRR& connections_inf,
        NetPinsMatrix<float>& net_delay,
        const ClusteredPinAtomPinsLookup& netlist_pin_lookup,
        std::shared_ptr<SetupHoldTimingInfo> timing_info,
        NetPinTimingInvalidator* pin_timing_invalidator,
        route_budgets& budgeting_inf,
        const RoutingPredictor& routing_predictor,
        const vtr::vector<ParentNetId, std::vector<std::unordered_map<RRNodeId, int>>>& choking_spots,
        bool is_flat)
        : _routers_th(_make_router(router_lookahead, is_flat))
        , _net_list(net_list)
        , _router_opts(router_opts)
        , _connections_inf(connections_inf)
        , _net_delay(net_delay)
        , _netlist_pin_lookup(netlist_pin_lookup)
        , _timing_info(timing_info)
        , _pin_timing_invalidator(pin_timing_invalidator)
        , _budgeting_inf(budgeting_inf)
        , _routing_predictor(routing_predictor)
        , _choking_spots(choking_spots)
        , _is_flat(is_flat)
        , _net_known_samples(net_list.nets().size())
        , _net_known_samples_m(net_list.nets().size())
        , _is_decomp_disabled(net_list.nets().size()) {}
    ~DecompNetlistRouter() {}

    /** Run a single iteration of netlist routing for this->_net_list. This usually means calling
     * \ref route_net for each net, which will handle other global updates.
     * \return RouteIterResults for this iteration. */
    RouteIterResults route_netlist(int itry, float pres_fac, float worst_neg_slack);
    void set_rcv_enabled(bool x);
    void set_timing_info(std::shared_ptr<SetupHoldTimingInfo> timing_info);

  private:
    /** Should we decompose this net? */
    bool should_decompose_net(ParentNetId net_id, const PartitionTreeNode& node);
    /** Get a bitset with sinks to route before net decomposition */
    vtr::dynamic_bitset<> get_decomposition_mask(ParentNetId net_id, const PartitionTreeNode& node);
    /** Get a bitset with sinks to route before virtual net decomposition */
    vtr::dynamic_bitset<> get_vnet_decomposition_mask(const VirtualNet& vnet, const PartitionTreeNode& node);
    /** Decompose and route a regular net. Output the resulting vnets to \p left and \p right. Return success status. */
    bool decompose_and_route_net(ParentNetId net_id, const PartitionTreeNode& node, VirtualNet& left, VirtualNet& right);
    /** Decompose and route a virtual net. Output the resulting vnets to \p left and \p right. Return success status. */
    bool decompose_and_route_vnet(VirtualNet& vnet, const PartitionTreeNode& node, VirtualNet& left, VirtualNet& right);
    /** A single task to route nets inside a PartitionTree node and add tasks for its child nodes to task group \p g. */
    void route_partition_tree_node(tbb::task_group& g, PartitionTreeNode& node);

    ConnectionRouter<HeapType> _make_router(const RouterLookahead* router_lookahead, bool is_flat) {
        auto& device_ctx = g_vpr_ctx.device();
        auto& route_ctx = g_vpr_ctx.mutable_routing();

        return ConnectionRouter<HeapType>(
            device_ctx.grid,
            *router_lookahead,
            device_ctx.rr_graph.rr_nodes(),
            &device_ctx.rr_graph,
            device_ctx.rr_rc_data,
            device_ctx.rr_graph.rr_switch(),
            route_ctx.rr_node_route_inf,
            is_flat);
    }

    /* Context fields */
    tbb::enumerable_thread_specific<ConnectionRouter<HeapType>> _routers_th;
    const Netlist<>& _net_list;
    const t_router_opts& _router_opts;
    CBRR& _connections_inf;
    tbb::enumerable_thread_specific<RouteIterResults> _results_th;
    NetPinsMatrix<float>& _net_delay;
    const ClusteredPinAtomPinsLookup& _netlist_pin_lookup;
    std::shared_ptr<SetupHoldTimingInfo> _timing_info;
    NetPinTimingInvalidator* _pin_timing_invalidator;
    route_budgets& _budgeting_inf;
    const RoutingPredictor& _routing_predictor;
    const vtr::vector<ParentNetId, std::vector<std::unordered_map<RRNodeId, int>>>& _choking_spots;
    bool _is_flat;

    /* Routing parameters for current iteration */
    int _itry;
    float _pres_fac;
    float _worst_neg_slack;

    /* Sinks to be always sampled for decomposition for each net */
    vtr::vector<ParentNetId, vtr::dynamic_bitset<>> _net_known_samples;
    vtr::vector<ParentNetId, std::mutex> _net_known_samples_m;

    /* Is decomposition disabled for this net? */
    vtr::vector<ParentNetId, bool> _is_decomp_disabled;
};

/** Maximum number of iterations for net decomposition */
const int MAX_DECOMP_ITER = 5;

#include "DecompNetlistRouter.tpp"
