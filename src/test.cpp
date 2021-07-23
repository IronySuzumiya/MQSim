#include <fstream>
#include <sstream>
#include <functional>

#include "host/Host_IO_Request.h"
#include "ssd/SSD_Defs.h"
#include "exec/Execution_Parameter_Set.h"
#include "exec/SSD_Device.h"
#include "exec/Host_System.h"
#include "utils/rapidxml/rapidxml.hpp"
#include "utils/DistributionTypes.h"
#include "sim/Sim_Defs.h"
#include "sim/EventTree.h"
#include "sim/Sim_Object.h"
#include "sim/Sim_Event.h"

#include "ssd/NVM_PHY_ONFI_NVDDR2.h"

class MQSimWrapper {
public:
  struct Address {
    uint32_t chanid;
    uint32_t chipid;
    uint32_t dieid;
    uint32_t planeid;
    uint32_t blockid;
    uint32_t pageid;
  };

private:
  Execution_Parameter_Set* _exec_params;
  SSD_Device* _ssd;
  Host_System* _host;

  uint32_t _sectors_per_page;

  void load_ssd_config(const std::string& ssd_config_file) {
    std::fstream fs;
    fs.open(ssd_config_file, std::ios::in);
    assert(fs);
    std::stringstream ss;
    ss << fs.rdbuf();
    std::string content(ss.str());
    rapidxml::xml_document<> doc;
    char* temp_string = new char[content.length() + 1];
    strcpy(temp_string, content.c_str());
    doc.parse<0>(temp_string);
    rapidxml::xml_node<> *mqsim_config = doc.first_node("Execution_Parameter_Set");
    assert(mqsim_config);
    _exec_params->XML_deserialize(mqsim_config);
    delete[] temp_string;
    fs.close();
  }

  void load_workload_config(const std::string& workload_config_file) {
    std::fstream fs;
    fs.open(workload_config_file, std::ios::in);
    assert(fs);
    std::stringstream ss;
    ss << fs.rdbuf();
    std::string content(ss.str());
    rapidxml::xml_document<> doc;
    char* temp_string = new char[content.length() + 1];
    strcpy(temp_string, content.c_str());
    doc.parse<0>(temp_string);
    rapidxml::xml_node<> *mqsim_io_scenarios = doc.first_node("MQSim_IO_Scenarios");  
    assert(mqsim_io_scenarios);
    auto xml_io_scenario = mqsim_io_scenarios->first_node("IO_Scenario");
    auto flow_def = xml_io_scenario->first_node();
    assert(!strcmp(flow_def->name(), "IO_Flow_Parameter_Set_Trace_Based"));
    auto flow = new IO_Flow_Parameter_Set_Trace_Based;
    flow->XML_deserialize(flow_def);
    _exec_params->Host_Configuration.IO_Flow_Definitions.push_back(flow);
    delete []temp_string;
    fs.close();
  }

public:
  MQSimWrapper() : _exec_params(new Execution_Parameter_Set), _ssd(nullptr), _host(nullptr) {
    Simulator->Reset();
    load_ssd_config("ssdconfig.xml");
    load_workload_config("dummy_workload.xml");
    _ssd = new SSD_Device(&_exec_params->SSD_Device_Configuration, &_exec_params->Host_Configuration.IO_Flow_Definitions);
    _exec_params->Host_Configuration.Input_file_path = "dummy_workload";
    _host = new Host_System(&_exec_params->Host_Configuration, _exec_params->SSD_Device_Configuration.Enabled_Preconditioning, _ssd->Host_interface);
    _host->Attach_ssd_device(_ssd);
    Simulator->get_ready();
    Simulator->clear_dummy_event();

    _sectors_per_page = _exec_params->SSD_Device_Configuration.Flash_Parameters.Page_Capacity / SECTOR_SIZE_IN_BYTE;
  }

  ~MQSimWrapper() {
    delete _exec_params;
    delete _ssd;
    delete _host;
  }

  void read(const std::vector<Address>& addrs, const uint32_t pages, std::function<void(void)> callback, bool local = true) {
    auto request = new SSD_Components::User_Request;
    request->Stream_id = 0;
    request->Priority_class = IO_Flow_Priority_Class::Priority::HIGH;
    request->STAT_InitiationTime = Simulator->Time();
    request->Type = SSD_Components::UserRequestType::READ;
    request->Start_LBA = 0; // not used
    request->SizeInSectors = _sectors_per_page * pages;
    request->Size_in_byte = _exec_params->SSD_Device_Configuration.Flash_Parameters.Page_Capacity * pages;

    uint32_t chanid = addrs.front().chanid;
    request->channel_busy_callback = [chanid](){ std::cout << "chan-" << chanid << " busy" << std::endl; };
    request->channel_idle_callback = [chanid](){ std::cout << "chan-" << chanid << " idle" << std::endl; };
    request->finish_callback = callback;
    request->local = local;

    for(auto& addr : addrs) {
      auto trans(
        new SSD_Components::NVM_Transaction_Flash_RD(
          SSD_Components::Transaction_Source_Type::USERIO, 0,
          request->Size_in_byte, 0, NO_PPA, // not used
          request, IO_Flow_Priority_Class::Priority::HIGH, 0, ~0UL, CurrentTimeStamp
        )
      );
      trans->Address.ChannelID = addr.chanid;
      trans->Address.ChipID = addr.chipid;
      trans->Address.DieID = addr.dieid;
      trans->Address.PlaneID = addr.planeid;
      trans->Address.BlockID = addr.blockid;
      trans->Address.PageID = addr.pageid;

      trans->Physical_address_determined = true;

      request->Transaction_list.push_back(trans);
    }

    auto&& tsu = dynamic_cast<SSD_Components::FTL*>(_ssd->Firmware)->TSU;
    if(request->Transaction_list.size() > 0) {
      tsu->Prepare_for_transaction_submit();
      for(const auto& trans : request->Transaction_list) {
        auto&& trans_flash = (SSD_Components::NVM_Transaction_Flash*)trans;
        assert(trans_flash->Physical_address_determined);
        tsu->Submit_transaction(trans_flash);
      }
      tsu->Schedule();
    }
  }

  void program(const std::vector<Address>& addrs, const uint32_t pages, std::function<void(void)> callback, bool local = true) {
    auto request = new SSD_Components::User_Request;
    request->Stream_id = 0;
    request->Priority_class = IO_Flow_Priority_Class::Priority::HIGH;
    request->STAT_InitiationTime = Simulator->Time();
    request->Type = SSD_Components::UserRequestType::WRITE;
    request->Start_LBA = 0; // not used
    request->SizeInSectors = _sectors_per_page * pages;
    request->Size_in_byte = _exec_params->SSD_Device_Configuration.Flash_Parameters.Page_Capacity * pages;

    uint32_t chanid = addrs.front().chanid;
    request->channel_busy_callback = [chanid](){ std::cout << "chan-" << chanid << " busy" << std::endl; };
    request->channel_idle_callback = [chanid](){ std::cout << "chan-" << chanid << " idle" << std::endl; };
    request->finish_callback = callback;
    request->local = local;

    for(auto& addr : addrs) {
      auto trans(
        new SSD_Components::NVM_Transaction_Flash_WR(
          SSD_Components::Transaction_Source_Type::USERIO, 0,
          request->Size_in_byte, 0, // not used
          request, IO_Flow_Priority_Class::Priority::HIGH, 0, ~0UL, CurrentTimeStamp
        )
      );
      trans->Address.ChannelID = addr.chanid;
      trans->Address.ChipID = addr.chipid;
      trans->Address.DieID = addr.dieid;
      trans->Address.PlaneID = addr.planeid;
      trans->Address.BlockID = addr.blockid;
      trans->Address.PageID = addr.pageid;

      trans->Physical_address_determined = true;

      request->Transaction_list.push_back(trans);
    }

    auto&& tsu = dynamic_cast<SSD_Components::FTL*>(_ssd->Firmware)->TSU;
    if(request->Transaction_list.size() > 0) {
      tsu->Prepare_for_transaction_submit();
      for(const auto& trans : request->Transaction_list) {
        auto&& trans_flash = (SSD_Components::NVM_Transaction_Flash*)trans;
        assert(trans_flash->Physical_address_determined);
        tsu->Submit_transaction(trans_flash);
        auto&& trans_flash_wr = (SSD_Components::NVM_Transaction_Flash_WR*)trans_flash;
        if(trans_flash_wr->RelatedRead != nullptr) {
          tsu->Submit_transaction(trans_flash_wr->RelatedRead);
        }
      }
      tsu->Schedule();
    }
  }

  /*void read(const std::vector<Address>& addrs, const uint32_t pages, std::function<void(void)> callback) {
    assert(pages > 0);
    if(pages == 1) read_one_page(addrs, callback);
    else {
      read_one_page(addrs, callback);
      std::vector<Address> per_addrs(addrs);
      for(uint32_t p = 1; p < pages; ++p) {
        for(auto& addr : per_addrs) {
          if(++addr.pageid == _exec_params->SSD_Device_Configuration.Flash_Parameters.Page_No_Per_Block) {
            addr.pageid = 0;
            assert(++addr.blockid < _exec_params->SSD_Device_Configuration.Flash_Parameters.Block_No_Per_Plane);
          }
        }
        read_one_page(per_addrs, callback);
      }
    }
  }

  void program(const std::vector<Address>& addrs, const uint32_t pages, std::function<void(void)> callback) {
    assert(pages > 0);
    if(pages == 1) program_one_page(addrs, callback);
    else {
      program_one_page(addrs, callback);
      std::vector<Address> per_addrs(addrs);
      for(uint32_t p = 1; p < pages; ++p) {
        for(auto& addr : per_addrs) {
          if(++addr.pageid == _exec_params->SSD_Device_Configuration.Flash_Parameters.Page_No_Per_Block) {
            addr.pageid = 0;
            assert(++addr.blockid < _exec_params->SSD_Device_Configuration.Flash_Parameters.Block_No_Per_Plane);
          }
        }
        program_one_page(per_addrs, callback);
      }
    }
  }

  void from_chip_to_board(const std::vector<Address>& addrs, const uint32_t pages, std::function<void(void)> callback) {
    assert(pages > 0);
    if(pages == 1) read_one_page(addrs, callback, false);
    else {
      read_one_page(addrs, callback, false);
      std::vector<Address> per_addrs(addrs);
      for(uint32_t p = 1; p < pages; ++p) {
        for(auto& addr : per_addrs) {
          if(++addr.pageid == _exec_params->SSD_Device_Configuration.Flash_Parameters.Page_No_Per_Block) {
            addr.pageid = 0;
            assert(++addr.blockid < _exec_params->SSD_Device_Configuration.Flash_Parameters.Block_No_Per_Plane);
          }
        }
        read_one_page(per_addrs, callback, false);
      }
    }
  }

  void from_board_to_chip(const std::vector<Address>& addrs, const uint32_t pages, std::function<void(void)> callback) {
    assert(pages > 0);
    if(pages == 1) program_one_page(addrs, callback, false);
    else {
      program_one_page(addrs, callback, false);
      std::vector<Address> per_addrs(addrs);
      for(uint32_t p = 1; p < pages; ++p) {
        for(auto& addr : per_addrs) {
          if(++addr.pageid == _exec_params->SSD_Device_Configuration.Flash_Parameters.Page_No_Per_Block) {
            addr.pageid = 0;
            assert(++addr.blockid < _exec_params->SSD_Device_Configuration.Flash_Parameters.Block_No_Per_Plane);
          }
        }
        program_one_page(per_addrs, callback, false);
      }
    }
  }*/

  void tick() {
    Simulator->tick();
  }

  void skip_to_next_event() {
    assert(get_next_event_firetime() > Simulator->Time());
    Simulator->set_sim_time(get_next_event_firetime() - 1);
    tick();
  }
  bool is_event_tree_empty() const {
    return Simulator->is_event_tree_empty();
  }
  uint64_t get_next_event_firetime() const {
    return Simulator->get_next_event_firetime();
  }
};

int main() {
  auto ssd(new MQSimWrapper);

  auto callback = []() {
    //std::cout << "callback" << std::endl;
  };

  std::vector<MQSimWrapper::Address> addrs;
  uint32_t b = 2, p = 0;

  addrs.push_back({ 0, 0, 0, 0, b, p });
  addrs.push_back({ 0, 0, 0, 1, b, p });
  addrs.push_back({ 0, 0, 0, 2, b, p });
  addrs.push_back({ 0, 0, 0, 3, b, p });
  ssd->program(addrs, 2, callback);

  addrs.clear();
  addrs.push_back({ 0, 0, 0, 0, b, p });
  addrs.push_back({ 0, 0, 0, 1, b, p });
  addrs.push_back({ 0, 0, 0, 2, b, p });
  addrs.push_back({ 0, 0, 0, 3, b, p });
  ssd->read(addrs, 4, callback);

  while(!ssd->is_event_tree_empty()) {
    ssd->skip_to_next_event();
  }

  /*for(uint32_t b = 0; b < 2048; ++b) {
    for(uint32_t p = 0; p < 256; ++p) {
      addrs.push_back({ 0, 0, 0, 0, b, p });
      addrs.push_back({ 0, 0, 0, 1, b, p });
      addrs.push_back({ 0, 0, 0, 2, b, p });
      addrs.push_back({ 0, 0, 0, 3, b, p });
      ssd->program(addrs, callback);

      addrs.clear();
      addrs.push_back({ 0, 0, 0, 0, b, p });
      addrs.push_back({ 0, 0, 0, 1, b, p });
      addrs.push_back({ 0, 0, 0, 2, b, p });
      addrs.push_back({ 0, 0, 0, 3, b, p });
      ssd->read(addrs, callback);

      addrs.clear();
      addrs.push_back({ 0, 0, 0, 0, b, p });
      addrs.push_back({ 0, 0, 0, 1, b, p });
      addrs.push_back({ 0, 0, 0, 2, b, p });
      addrs.push_back({ 0, 0, 0, 3, b, p });
      ssd->read(addrs, callback);

      while(!ssd->is_event_tree_empty()) {
        ssd->skip_to_next_event();
      }
    }
  }*/

  std::cout << "finish @ " << Simulator->Time() << " ns" << std::endl;

  /*addrs.clear();
  addrs.push_back({ 0, 0, 0, 0, 1, 0 });
  addrs.push_back({ 0, 0, 0, 1, 1, 0 });
  addrs.push_back({ 0, 0, 0, 2, 1, 0 });
  addrs.push_back({ 0, 0, 0, 3, 1, 0 });
  ssd->program(addrs, callback);

  while(!ssd->is_event_tree_empty()) {
    ssd->skip_to_next_event();
  }*/

  delete ssd;

  return 0;
}
