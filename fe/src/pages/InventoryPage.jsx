import React, { useState, useEffect } from 'react';
import Layout from '../components/Layout/Layout';
import InventoryList from '../components/Inventory/InventoryList';
import InventoryForm from '../components/Inventory/InventoryForm';
import Modal from 'react-modal';
import { useInventory } from '../hooks/useInventory';
import '../styles/InventoryStyles.css';

const customModalStyles = {
  content: {
    top: '50%',
    left: '50%',
    right: 'auto',
    bottom: 'auto',
    marginRight: '-50%',
    transform: 'translate(-50%, -50%)',
    maxWidth: '500px',
    width: '90%',
    padding: '2rem',
    borderRadius: '0.5rem',
    maxHeight: '90vh',
    overflow: 'auto'
  },
  overlay: {
    backgroundColor: 'rgba(0, 0, 0, 0.75)'
  }
};

function InventoryPage() {
  const [isModalOpen, setIsModalOpen] = useState(false);
  const { fetchInventory } = useInventory();
  
  // 컴포넌트 마운트 시 재고 데이터 로드
  useEffect(() => {
    fetchInventory();
  }, [fetchInventory]);

  const openModal = () => {
    setIsModalOpen(true);
  };

  const closeModal = () => {
    setIsModalOpen(false);
  };

  return (
    <Layout>
      <div className="inventory-page">
        <div className="inventory-header-container">
          <h1 className="inventory-title">재고 관리</h1>
        </div>

        <InventoryList onAddInventory={openModal} />

        <Modal
          isOpen={isModalOpen}
          onRequestClose={closeModal}
          style={customModalStyles}
          contentLabel="새 재고 아이템 추가"
        >
          <InventoryForm onClose={closeModal} />
        </Modal>
      </div>
    </Layout>
  );
}

export default InventoryPage;
